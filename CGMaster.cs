using System.Collections.Concurrent;
using Gurobi;

namespace MCFStatic
{
    // Column-generation formulation mode.
    //   Leg        : only the global per-arc capacity constraints (5a).
    //   LegRequest : additionally, for every (request r, resource s) touched by at
    //                least one of r's paths, sum_{p of r using s} x[r,p] <= y[s] (5b).
    internal enum CGMode
    {
        Leg,
        LegRequest,
    }

    // Restricted master problem for column generation (stronger formulation):
    //
    //   min  sum_s sigma_s * y_s + sum_r sum_p Cost_p^r * lambda_p^r
    //   s.t. sum_r sum_p v_r * iota_pa^r * lambda_p^r <= c_a * y_{s(a)}   [rho_a]   (5a)
    //        sum_p iota_ps^r * lambda_p^r <= y_s, r in R, s in S^r       [mu_rs]   (5b)
    //        sum_p lambda_p^r = 1                                        [omega_r] (5c)
    //        y_s <= 1                                                    [eta_s]   (5d)
    //
    // Variables:
    //   x[r, p] : continuous path-selection variable (lambda_p^r)
    //   y[s]    : continuous resource-activation variable
    //
    // Arcs without a resource (artificial dummy arcs) have a constant capacity
    // row usage <= c_a. The (5b) rows are added lazily: only (r, s) pairs used by
    // a column of r are in the model, and idle rows are pruned. The duals of the
    // missing rows are estimated (see ActualResReqDual).
    internal sealed class CGMaster : IDisposable
    {
        private readonly GRBEnv _env;
        private readonly GRBModel _model;
        private readonly Network _network;

        // x[(requestId, pathId)] -> path-selection variable.
        private readonly Dictionary<(string Request, string Path), GRBVar> _x = new();
        // y[resourceId] -> resource-activation variable.
        private readonly Dictionary<string, GRBVar> _y = new(StringComparer.Ordinal);
        // Capacity row per arc (5a).
        private readonly Dictionary<string, GRBConstr> _capconstr = new(StringComparer.Ordinal);
        private readonly Dictionary<string, GRBConstr> _convexconstr = new(StringComparer.Ordinal);

        // Explicit y[s] <= 1 constraint (5d) and its dual, keyed by resource id.
        private readonly Dictionary<string, GRBConstr> _yboundconstr = new(StringComparer.Ordinal);
        private readonly Dictionary<string, double> _yboundDuals = new(StringComparer.Ordinal);

        // Branching lower bound y[s] >= l as an explicit row so its dual is readable.
        private readonly Dictionary<string, GRBConstr> _ylowerconstr = new(StringComparer.Ordinal);
        private readonly Dictionary<string, double> _ylowerDuals = new(StringComparer.Ordinal);

        // Snapshot of each y[s] value after the last solve.
        private readonly Dictionary<string, double> _yValues = new(StringComparer.Ordinal);

        // Dual values keyed by arc id (capacity) and request id (convexity).
        private readonly Dictionary<string, double> _capDuals = new(StringComparer.Ordinal);
        private readonly Dictionary<string, double> _convexDuals = new(StringComparer.Ordinal);

        // Stabilized mirrors of the dual values.
        private readonly Dictionary<string, double> _capDuals_stable = new(StringComparer.Ordinal);
        private readonly Dictionary<string, double> _convexDuals_stable = new(StringComparer.Ordinal);

        // LegRequest mode only: per-(request, resource) linking constraint and its dual.
        private readonly CGMode _mode;
        private readonly Dictionary<(string Request, string Resource), GRBConstr> _resReqConstr = new();
        private readonly Dictionary<(string Request, string Resource), double> _resReqDuals = new();
        private readonly Dictionary<(string Request, string Resource), double> _resReqDuals_stable = new();

        // LegRequest mode: per-resource sum of the resource-request duals whose
        // linking constraint currently exists in the formulation (Q_s).
        private readonly Dictionary<string, double> _resReqDualSumByResource = new(StringComparer.Ordinal);

        // LegRequest mode: number of active columns that touch each (request, resource) pair.
        private readonly Dictionary<(string Request, string Resource), int> _resReqUsage = new();

        // LegRequest mode: for each (request, resource) pair with no column left, how
        // many UpdatePaths rounds it has stayed unused.
        private readonly Dictionary<(string Request, string Resource), int> _resReqIdle = new();

        // Rounds a pair may stay unused before its linking constraint is dropped:
        // 0 removes it in the same round, negative never removes it.
        private readonly int _resReqIdleLimit;

        // Consecutive cleaning rounds each column has been unused (X == 0 with a
        // positive reduced cost). Removing a column drops its entry, so a re-added
        // column starts with a fresh clock.
        private readonly Dictionary<(string Request, string Path), int> _columnIdle = new();

        // Extra rounds an unused column stays in the master before it is removed:
        // 0 removes it the first time it is unused, negative never removes it.
        private readonly int _columnIdleLimit;

        // Missing resource-request dual estimate policy (see ActualResReqDual):
        //   0 : no estimate (absent pairs get 0)
        //   1 : split the y residual equally over all absent pairs on the resource
        //   2 : hybrid -- 0 until the Lagrangian bound improves by less than
        //       HybridSwitchTolerance over the last HybridSwitchStallIterations iterations, then 1.
        //   3 : alternate -- like 2, then back to 0 when the master LP fails to
        //       decrease by the same tolerance/window; repeat until convergence.
        //   4 : reverse hybrid -- 1 until the master LP objective decreases by less than
        //       HybridSwitchTolerance over the last HybridSwitchStallIterations iterations, then 0.
        private readonly int _enable_dual_lb = 1;
        private bool _hybridSwitched;

        // Distinct resources touched by each column.
        private readonly Dictionary<(string Request, string Path), string[]> _xResources = new();

        // Objective coefficient (sigma_s) of each y[s] outside a node-level Phase 1.
        private readonly Dictionary<string, double> _yCost = new(StringComparer.Ordinal);
        private bool _phase1Objective;

        public CGMaster(GRBEnv env, Dictionary<string,Request> requests, Dictionary<string,RoutePath> pathdict, Network network, bool includeArcFixedCost = false, CGMode mode = CGMode.Leg, int enable_dual_lb = 1, int legReqIdleLimit = 2, double hybridSwitchTolerance = 0.01, int hybridSwitchStallIterations = 5, int columnIdleLimit = 0)
        {
            if (enable_dual_lb is < 0 or > 4)
                throw new ArgumentOutOfRangeException(nameof(enable_dual_lb), "Expected 0, 1, 2, 3, or 4.");
            if (!double.IsFinite(hybridSwitchTolerance) || hybridSwitchTolerance <= 0d)
                throw new ArgumentOutOfRangeException(nameof(hybridSwitchTolerance), "Must be finite and positive.");
            if (hybridSwitchStallIterations <= 0)
                throw new ArgumentOutOfRangeException(nameof(hybridSwitchStallIterations), "Must be positive.");
            HybridSwitchTolerance = hybridSwitchTolerance;
            HybridSwitchStallIterations = hybridSwitchStallIterations;
            // Reuse the caller's environment (single-use license: only one env).
            _env = env;
            _mode = mode;
            _network = network;

            _model = new GRBModel(_env)
            {
                ModelName = "CGMaster",
            };
            _model.Parameters.Method = ExecutionSettings.LpMethod;
            // _model.Parameters.Crossover = 4;
            _enable_dual_lb = enable_dual_lb;
            _resReqIdleLimit = legReqIdleLimit;
            _columnIdleLimit = columnIdleLimit;

            var capusageexpr = new Dictionary<string, GRBLinExpr>(StringComparer.Ordinal);

            // LegRequest mode: sum of a request's path vars that touch a given resource.
            var resRequsage = new Dictionary<(string Request, string Resource), GRBLinExpr>();

            foreach (string r in requests.Keys)
            {
                GRBLinExpr requestconvexexpr = new GRBLinExpr();
                foreach (string p in requests[r].Paths)
                {
                    _x[(r, p)] = _model.AddVar(0d, GRB.INFINITY, pathdict[p].Cost, GRB.CONTINUOUS, $"x[{r},{p}]");
                    requestconvexexpr.AddTerm(1d, _x[(r, p)]);
                    var resIds = ResourcesOf(pathdict[p]);
                    _xResources[(r, p)] = resIds;
                    foreach (Arc a in pathdict[p].Arcs)
                    {
                        capusageexpr.TryAdd(a.Id, new GRBLinExpr());
                        capusageexpr[a.Id].AddTerm(requests[r].DemandGbps, _x[(r, p)]);
                    }

                    if (_mode == CGMode.LegRequest)
                    {
                        foreach (var resId in resIds)
                        {
                            var key = (r, resId);
                            resRequsage.TryAdd(key, new GRBLinExpr());
                            resRequsage[key].AddTerm(1d, _x[(r, p)]);
                            _resReqUsage[key] = _resReqUsage.GetValueOrDefault(key) + 1;
                        }
                    }
                }
                _convexconstr[r] = _model.AddConstr(requestconvexexpr == 1d, $"convex[{r}]");
            }

            foreach (var res in network.Resources)
            {
                // Phase 1 keeps resources free (includeArcFixedCost = false); Phase 2
                // charges sigma_s.
                var yFixed = includeArcFixedCost ? res.FixedCost : 0d;
                _yCost[res.Id] = yFixed;
                _y[res.Id] = _model.AddVar(0d, GRB.INFINITY, yFixed, GRB.CONTINUOUS, $"y[{res.Id}]");

                var ybound = new GRBLinExpr();
                ybound.AddTerm(1d, _y[res.Id]);
                _yboundconstr[res.Id] = _model.AddConstr(ybound <= 1d, $"ybound[{res.Id}]");

                var ylower = new GRBLinExpr();
                ylower.AddTerm(1d, _y[res.Id]);
                _ylowerconstr[res.Id] = _model.AddConstr(ylower >= 0d, $"ylower[{res.Id}]");
            }

            foreach (var arc in network.Arcs)
            {
                if (capusageexpr.TryGetValue(arc.Id, out var usage))
                    _capconstr[arc.Id] = AddCapacityConstr(arc, usage);
            }

            // LegRequest mode: one linking constraint per (request, resource) that the
            // request actually uses: sum_{p of r using s} x[r,p] <= y[s].
            if (_mode == CGMode.LegRequest)
            {
                foreach (var ((r, resId), expr) in resRequsage)
                {
                    if (_y.TryGetValue(resId, out var yVar))
                    {
                        var rhs = new GRBLinExpr();
                        rhs.AddTerm(1d, yVar);
                        _resReqConstr[(r, resId)] = _model.AddConstr(expr <= rhs, $"resreq[{r},{resId}]");
                    }
                }
            }
        }

        // (5a): usage <= c_a * y[s(a)], or usage <= c_a for arcs without a resource.
        private GRBConstr AddCapacityConstr(Arc arc, GRBLinExpr usage)
        {
            if (arc.ResourceId is not null && _y.TryGetValue(arc.ResourceId, out var yVar))
            {
                var cap = new GRBLinExpr();
                cap.AddTerm(arc.CapacityGbps, yVar);
                return _model.AddConstr(usage <= cap, $"cap[{arc.Id}]");
            }
            return _model.AddConstr(usage <= arc.CapacityGbps, $"cap[{arc.Id}]");
        }

        // Distinct resource ids touched by a path (iota_ps = 1).
        private static string[] ResourcesOf(RoutePath path)
            => path.Arcs.Where(a => a.ResourceId is not null)
                        .Select(a => a.ResourceId!)
                        .Distinct(StringComparer.Ordinal)
                        .ToArray();

        public GRBModel Model => _model;

        // Gurobi optimization status of the last Optimize() call.
        public int Status => _model.Status;

        public Dictionary<(string Request, string Path), GRBVar> X => _x;

        // Resource-activation variables keyed by resource id.
        public Dictionary<string, GRBVar> Y => _y;

        // Dual value of each capacity constraint, keyed by arc id.
        public Dictionary<string, double> CapacityDuals => _capDuals;

        // Dual value of each convexity constraint, keyed by request id.
        public Dictionary<string, double> ConvexityDuals => _convexDuals;

        // Dual value of each explicit y[s] <= 1 constraint, keyed by resource id.
        public Dictionary<string, double> YBoundDuals => _yboundDuals;

        // Dual value of each branching y[s] >= l constraint, keyed by resource id.
        public Dictionary<string, double> YLowerDuals => _ylowerDuals;

        // Current branching bounds on y[s].
        public (double Lower, double Upper) GetResourceBounds(string resourceId)
        {
            if (!_y.ContainsKey(resourceId))
                throw new ArgumentException($"Resource '{resourceId}' has no activation variable.", nameof(resourceId));
            return (_ylowerconstr[resourceId].RHS, _yboundconstr[resourceId].RHS);
        }

        // Branch-and-price bound change on y[s], applied to explicit rows.
        public void SetResourceBounds(string resourceId, double lower, double upper)
        {
            if (!_y.ContainsKey(resourceId))
                throw new ArgumentException($"Resource '{resourceId}' has no activation variable.", nameof(resourceId));
            if (lower > upper)
                throw new ArgumentOutOfRangeException(nameof(lower), "lower bound exceeds upper bound.");

            _ylowerconstr[resourceId].RHS = lower;
            _yboundconstr[resourceId].RHS = upper;
            _model.Update();
        }

        // Contribution of the bound rows to the Lagrangian dual bound:
        // sum_s (u_s * eta_s + l_s * pi^lb_s).
        public double ArcBoundDualContribution()
        {
            var total = 0d;
            foreach (var (resId, pi) in _yboundDuals)
                total += _yboundconstr[resId].RHS * pi;
            foreach (var (resId, pi) in _ylowerDuals)
                total += _ylowerconstr[resId].RHS * pi;
            // Arcs without a resource have a constant capacity RHS c_a.
            foreach (var (arcId, pi) in _capDuals)
            {
                var constr = _capconstr[arcId];
                total += constr.RHS * pi;
            }
            return total;
        }

        // Solves the current restricted master problem and refreshes the duals.
        public void Optimize()
        {
            _model.Optimize();

            if (_model.Status != GRB.Status.OPTIMAL)
            {
                Optimization.ReportIfInfeasible(_model, "CGMaster");
                return;
            }

            _capDuals.Clear();
            foreach (var (arcId, constr) in _capconstr)
                _capDuals[arcId] = constr.Pi;

            _convexDuals.Clear();
            foreach (var (requestId, constr) in _convexconstr)
                _convexDuals[requestId] = constr.Pi;

            _yboundDuals.Clear();
            foreach (var (resId, constr) in _yboundconstr)
                _yboundDuals[resId] = constr.Pi;

            _ylowerDuals.Clear();
            foreach (var (resId, constr) in _ylowerconstr)
                _ylowerDuals[resId] = constr.Pi;

            _yValues.Clear();
            foreach (var (resId, var) in _y)
                _yValues[resId] = var.X;

            if (_mode == CGMode.LegRequest)
            {
                _resReqDuals.Clear();
                _resReqDualSumByResource.Clear();
                foreach (var (key, constr) in _resReqConstr)
                {
                    var pi = constr.Pi;
                    _resReqDuals[key] = pi;
                    // Accumulate Q_s: sum of existing resource-request duals on s.
                    _resReqDualSumByResource[key.Resource] = _resReqDualSumByResource.GetValueOrDefault(key.Resource) + pi;
                }
            }
        }

        public int EffectiveDualLbPolicy =>
            _enable_dual_lb switch
            {
                2 or 3 => _hybridSwitched ? 1 : 0,
                4 => _hybridSwitched ? 0 : 1,
                _ => _enable_dual_lb,
            };

        public int EnableDualLb => _enable_dual_lb;

        public bool HybridDualLbPending => _enable_dual_lb is 2 or 4 && !_hybridSwitched;

        public double HybridSwitchTolerance { get; }

        public int HybridSwitchStallIterations { get; }

        public bool SwitchHybridDualLb()
        {
            if (_enable_dual_lb != 3 && !HybridDualLbPending)
                return false;
            _hybridSwitched = !_hybridSwitched;
            return true;
        }

        private const double DualStabilityAlpha = 1.0;

        private const double YPositiveTol = 1e-6;

        private const double ResReqDualTol = 1e-7;

        public void SaveStableDuals()
        {
            _capDuals_stable.Clear();
            foreach (var (arcId, value) in _capDuals)
                _capDuals_stable[arcId] = value;

            _convexDuals_stable.Clear();
            foreach (var (requestId, value) in _convexDuals)
                _convexDuals_stable[requestId] = value;

            if (_mode == CGMode.LegRequest)
            {
                _resReqDuals_stable.Clear();
                foreach (var (key, value) in _resReqDuals)
                    _resReqDuals_stable[key] = value;
            }
        }

        public void ClearStableDuals()
        {
            _capDuals_stable.Clear();
            _convexDuals_stable.Clear();
            _resReqDuals_stable.Clear();
        }

        // Node-level Phase 1: real columns and y[s] cost 0, artificial columns keep Cost.
        public void SetPhase1Objective(bool enabled, Dictionary<string, RoutePath> pathdict)
        {
            _phase1Objective = enabled;
            foreach (var ((_, pathId), xVar) in _x)
                xVar.Obj = ObjectiveCoefficient(pathdict[pathId]);
            foreach (var (resId, yVar) in _y)
                yVar.Obj = enabled ? 0d : _yCost[resId];
            _model.Update();
        }

        private double ObjectiveCoefficient(RoutePath path)
            => _phase1Objective && path.Arcs.Count > 0 ? 0d : path.Cost;

        private double CombinedCapDual(string arcId)
        {
            var actual = _capDuals.TryGetValue(arcId, out var a) ? a : 0d;
            if (!_capDuals_stable.TryGetValue(arcId, out var stable))
                return actual;
            return DualStabilityAlpha * actual + (1d - DualStabilityAlpha) * stable;
        }

        public double CombinedConvexDual(string requestId)
        {
            var actual = _convexDuals.TryGetValue(requestId, out var a) ? a : 0d;
            if (!_convexDuals_stable.TryGetValue(requestId, out var stable))
                return actual;
            return DualStabilityAlpha * actual + (1d - DualStabilityAlpha) * stable;
        }

        private double CombinedResReqDual((string Request, string Resource) key, Resource res, double absentWeightOnResource)
        {
            var actual = ActualResReqDual(key, res, absentWeightOnResource);
            if (!_resReqDuals_stable.TryGetValue(key, out var stable))
                return actual;
            return DualStabilityAlpha * actual + (1d - DualStabilityAlpha) * stable;
        }

        // Actual (current master) resource-request dual mu_{r,s}.
        //
        // If the linking constraint exists, the optimizer's dual is returned.
        // Otherwise, when y[s] > 0 the row would be slack so the dual is 0; when
        // y[s] = 0 the residual of y[s]'s reduced cost
        //     R_s = eta_s + lowerDual_s - sum_{a in s} c_a * rho_a - sigma_s - Q_s
        // is split over the absent pairs: mu_{r,s} = R_s / W_s, which
        // drives y[s]'s reduced cost to zero.
        private double ActualResReqDual((string Request, string Resource) key, Resource res, double absentWeightOnResource)
        {
            if (_resReqDuals.TryGetValue(key, out var mu))
            {
                return mu;
            }
            else if (EffectiveDualLbPolicy > 0)
            {
                var yVal = _yValues.TryGetValue(res.Id, out var yv) ? yv : 0d;
                if (yVal > YPositiveTol || absentWeightOnResource <= 0d)
                    return 0d;

                var yboundDual = _yboundDuals.TryGetValue(res.Id, out var yb) ? yb : 0d;
                var ylowerDual = _ylowerDuals.TryGetValue(res.Id, out var yl) ? yl : 0d;
                var capTerm = 0d;
                foreach (var arc in res.Arcs)
                    capTerm += arc.CapacityGbps * (_capDuals.TryGetValue(arc.Id, out var cd) ? cd : 0d);
                var qSum = _resReqDualSumByResource.TryGetValue(res.Id, out var q) ? q : 0d;
                var yCost = _phase1Objective ? 0d : _yCost[res.Id];
                var est = (yboundDual + ylowerDual - capTerm - yCost - qSum) / absentWeightOnResource;

                return Math.Min(est, 0d);
            }
            else
            {
                return 0;
            }
        }

        // Reduced cost of a path under the ACTUAL duals:
        //   sum_a (arcCost_a - rho_a * v_r) - sum_{s touched} mu_{r,s} - omega_r.
        public double ActualReducedCost(Request request, IReadOnlyList<Arc> arcs, bool includeArcCost = true)
        {
            ArgumentNullException.ThrowIfNull(request);
            ArgumentNullException.ThrowIfNull(arcs);

            var vol = request.DemandGbps;
            var w = 0d;
            var seen = new HashSet<string>(StringComparer.Ordinal);
            foreach (var arc in arcs)
            {
                var pi = _capDuals.TryGetValue(arc.Id, out var p) ? p : 0d;
                var isDummy = arc.LinkId.StartsWith("dummy[", StringComparison.Ordinal);
                var arcCost = (includeArcCost || isDummy)
                    ? arc.UsageCostPerGbps * vol
                    : 0d;
                w += arcCost - pi * vol;

                if (_mode == CGMode.LegRequest && arc.ResourceId is not null && seen.Add(arc.ResourceId) &&
                    _resReqDuals.TryGetValue((request.Id, arc.ResourceId), out var mu))
                    w += -mu;
            }

            var sigma = _convexDuals.TryGetValue(request.Id, out var sg) ? sg : 0d;
            return w - sigma;
        }

        public void Dispose()
        {
            _model.Dispose();
            foreach (var env in _ownedPricingEnvs)
                env.Dispose();
            _ownedPricingEnvs.Clear();
        }

        // Worker environments for parallel pricing MIPs (a Gurobi env must not be
        // used from several threads). Created lazily, reused across CG iterations,
        // disposed with the master.
        private readonly ConcurrentBag<GRBEnv> _pricingEnvPool = new();
        private readonly List<GRBEnv> _ownedPricingEnvs = new();
        private bool _parallelPricingDisabled;

        private GRBEnv RentPricingEnv()
        {
            if (_pricingEnvPool.TryTake(out var env))
                return env;
            env = ExecutionSettings.CreateGurobiEnvironment(threads: 1);
            lock (_ownedPricingEnvs)
                _ownedPricingEnvs.Add(env);
            return env;
        }

        // Column cleaning: ages every column against the last master solve and
        // returns the path ids of those unused (X == 0 and RC > rcTol) for more than
        // _columnIdleLimit consecutive rounds. A used column, or one for which keep
        // returns true, has its clock reset. Call once per cleaning round, after
        // Optimize().
        public List<string> CollectIdleColumns(double rcTol, Func<string, bool>? keep = null)
        {
            var idle = new List<string>();
            foreach (var (key, xVar) in _x)
            {
                if (keep is not null && keep(key.Path))
                {
                    _columnIdle.Remove(key);
                    continue;
                }

                if (xVar.X == 0 && xVar.RC > rcTol)
                {
                    var rounds = _columnIdle.GetValueOrDefault(key) + 1;
                    _columnIdle[key] = rounds;
                    if (_columnIdleLimit >= 0 && rounds > _columnIdleLimit)
                        idle.Add(key.Path);
                }
                else
                {
                    _columnIdle.Remove(key);
                }
            }
            return idle;
        }

        // Adds and removes paths from the model by their path ids.
        public void UpdatePaths(
            List<string> addPaths,
            List<string> removePaths,
            Dictionary<string, RoutePath> pathdict)
        {
            if (removePaths is not null)
            {
                foreach (var pathId in removePaths)
                {
                    var path = pathdict[pathId];
                    var requestId = path.Request.Id;
                    var key = (requestId, pathId);
                    if (_x.TryGetValue(key, out var var))
                    {
                        _model.Remove(var);
                        _x.Remove(key);
                        _columnIdle.Remove(key);
                        _xResources.Remove(key, out var resIds);

                        // LegRequest mode: drop this column's contribution to each
                        // (request, resource) usage count; zero starts the idle clock.
                        if (_mode == CGMode.LegRequest && resIds is not null)
                        {
                            foreach (var resId in resIds)
                            {
                                var rsKey = (requestId, resId);
                                if (!_resReqUsage.TryGetValue(rsKey, out var count))
                                    continue;

                                count--;
                                _resReqUsage[rsKey] = count;
                                if (count <= 0)
                                    _resReqIdle.TryAdd(rsKey, 0);
                            }
                        }
                    }
                }
            }

            if (addPaths is not null)
            {
                foreach (var pathId in addPaths)
                {
                    var path = pathdict[pathId];
                    var requestId = path.Request.Id;
                    var key = (requestId, pathId);
                    if (_x.ContainsKey(key))
                        continue;

                    var xVar = _model.AddVar(0d, 1d, ObjectiveCoefficient(path), GRB.CONTINUOUS, $"x[{requestId},{pathId}]");
                    _x[key] = xVar;
                    var resIds = ResourcesOf(path);
                    _xResources[key] = resIds;
                    _model.Update();

                    if (_convexconstr.TryGetValue(requestId, out var convex))
                        _model.ChgCoeff(convex, xVar, 1d);

                    foreach (var arc in path.Arcs)
                    {
                        if (_capconstr.TryGetValue(arc.Id, out var constr))
                        {
                            _model.ChgCoeff(constr, xVar, path.Request.DemandGbps);
                        }
                        else
                        {
                            var usage = new GRBLinExpr();
                            usage.AddTerm(path.Request.DemandGbps, xVar);
                            _capconstr[arc.Id] = AddCapacityConstr(arc, usage);
                        }
                    }

                    // LegRequest mode: link this column into each (request, resource)
                    // constraint, creating it the first time r uses s.
                    if (_mode == CGMode.LegRequest)
                    {
                        foreach (var resId in resIds)
                        {
                            var rsKey = (requestId, resId);
                            if (_resReqConstr.TryGetValue(rsKey, out var rs))
                            {
                                _model.ChgCoeff(rs, xVar, 1d);
                                _resReqUsage[rsKey] = Math.Max(_resReqUsage.GetValueOrDefault(rsKey), 0) + 1;
                                _resReqIdle.Remove(rsKey);
                            }
                            else if (_y.TryGetValue(resId, out var yVar))
                            {
                                var lhs = new GRBLinExpr();
                                lhs.AddTerm(1d, xVar);
                                var rhs = new GRBLinExpr();
                                rhs.AddTerm(1d, yVar);
                                _resReqConstr[rsKey] = _model.AddConstr(lhs <= rhs, $"resreq[{requestId},{resId}]");
                                _resReqUsage[rsKey] = Math.Max(_resReqUsage.GetValueOrDefault(rsKey), 0) + 1;
                                _resReqIdle.Remove(rsKey);
                            }
                        }
                    }
                }
            }

            if (_mode == CGMode.LegRequest && _resReqIdleLimit >= 0 && _resReqIdle.Count > 0)
                PruneIdleResourceRequestConstraints();

            _model.Update();
        }

        // Ages every unused (request, resource) pair by one round and drops the
        // linking constraints unused for _resReqIdleLimit rounds -- only when their
        // dual is zero (same rule as the former leg-request rows).
        private void PruneIdleResourceRequestConstraints()
        {
            var idleKeys = new List<(string Request, string Resource)>(_resReqIdle.Keys);
            foreach (var key in idleKeys)
            {
                var rounds = _resReqIdle[key];
                if (rounds < _resReqIdleLimit)
                {
                    _resReqIdle[key] = rounds + 1;
                    continue;
                }

                if (_resReqDuals.TryGetValue(key, out var mu) && Math.Abs(mu) > ResReqDualTol)
                    continue;

                _resReqIdle.Remove(key);
                _resReqUsage.Remove(key);
                if (_resReqConstr.Remove(key, out var rs))
                    _model.Remove(rs);
                _resReqDuals.Remove(key);
                _resReqDuals_stable.Remove(key);
            }
        }

        // Pricing sub-problem (Fig. 6), solved as a MIP for every request on its own
        // sub-network:
        //
        //   min  sum_a (theta_ar - rho_a * v_r) x_a - sum_s mu_rs * w_s  (- omega_r)
        //   s.t. flow conservation from o_r to d_r                          (6a)
        //        x_a <= w_{s(a)}                                            (6b)
        //        x, w binary
        //
        // Returns, per request id, up to n paths from the MIP solution pool whose
        // reduced cost (TotalCost - omega_r) is negative, best-first. TotalCost is
        // the path cost excluding -omega_r, matching the former Dijkstra output.
        public Dictionary<string, List<ShortestPath>> SolveSubproblem(
            Dictionary<string, Request> requests,
            int n = 1,
            bool includeArcCost = true,
            double toleranceFraction = 0d,
            bool randomizedPricing = false,
            IReadOnlySet<string>? forbiddenArcs = null,
            bool heuristic = false,
            bool warmStartMip = true,
            bool originalNetworkHeuristic = false)
        {
            ArgumentNullException.ThrowIfNull(requests);
            if (n <= 0)
                throw new ArgumentOutOfRangeException(nameof(n), "Number of paths must be positive.");

            // Resources reachable in each request's sub-network.
            var subResources = new Dictionary<string, HashSet<string>>(StringComparer.Ordinal);
            foreach (var req in requests.Values)
            {
                if (req.SubNetwork is null)
                    continue;
                var set = new HashSet<string>(StringComparer.Ordinal);
                foreach (var arc in req.SubNetwork.Network.Arcs)
                    if (arc.ResourceId is not null && _y.ContainsKey(arc.ResourceId))
                        set.Add(arc.ResourceId);
                subResources[req.Id] = set;
            }

            // LegRequest mode: denominator W_s of the missing-dual split -- the total
            // number of (request, resource) pairs whose sub-network contains
            // s but whose linking row is absent from the master.
            var resAbsentWeight = new Dictionary<string, double>(StringComparer.Ordinal);
            if (_mode == CGMode.LegRequest && EffectiveDualLbPolicy > 0)
            {
                foreach (var (reqId, set) in subResources)
                {
                    foreach (var resId in set)
                    {
                        if (_resReqConstr.ContainsKey((reqId, resId)))
                            continue;
                        resAbsentWeight[resId] = resAbsentWeight.GetValueOrDefault(resId) + 1d;
                    }
                }
            }

            var reqList = requests.Values.Where(r => r.SubNetwork is not null).ToList();
            var useMip = !originalNetworkHeuristic && !heuristic;

            // Heuristic pricing (no Gurobi, read-only duals) runs in parallel. Its
            // paths also serve as MIP warm starts.
            var heuristicPaths = new List<ShortestPath>[reqList.Count];
            if (!useMip || warmStartMip)
            {
                Parallel.For(0, reqList.Count, ExecutionSettings.ParallelOptions, i =>
                {
                    var req = reqList[i];
                    var (arcWeight, resourceWeight) = PricingWeights(req);
                    heuristicPaths[i] = originalNetworkHeuristic
                        ? SolvePricingOriginalNetwork(req, arcWeight, resourceWeight, subResources[req.Id], forbiddenArcs)
                        : SolvePricingCss(req, arcWeight, resourceWeight, subResources[req.Id], forbiddenArcs);
                });
            }

            // Pricing MIPs in parallel, one Gurobi env per worker. Falls back to the
            // sequential loop below if a worker env cannot be created (e.g. license limit).
            var mipPaths = new List<ShortestPath>?[reqList.Count];
            var mipWorkers = ExecutionSettings.SingleThreaded ? 0 : ExecutionSettings.PricingMipWorkers;
            if (useMip && mipWorkers > 0 && reqList.Count > 1 && !_parallelPricingDisabled)
            {
                try
                {
                    Parallel.For(0, reqList.Count,
                        new ParallelOptions { MaxDegreeOfParallelism = mipWorkers },
                        RentPricingEnv,
                        (i, _, env) =>
                        {
                            var req = reqList[i];
                            var (arcWeight, resourceWeight) = PricingWeights(req);
                            mipPaths[i] = SolvePricingMip(env, req, n, arcWeight, resourceWeight,
                                subResources[req.Id], forbiddenArcs,
                                warmStartMip ? heuristicPaths[i].FirstOrDefault() : null);
                            return env;
                        },
                        env => _pricingEnvPool.Add(env));
                }
                catch (AggregateException ex) when (ex.InnerExceptions.All(e => e is GRBException))
                {
                    Console.Error.WriteLine($"Parallel pricing MIP disabled, falling back to sequential: {ex.InnerExceptions[0].Message}");
                    _parallelPricingDisabled = true;
                    Array.Clear(mipPaths);
                }
            }

            // Results are assembled in request order so column insertion stays deterministic.
            var result = new Dictionary<string, List<ShortestPath>>(StringComparer.Ordinal);
            for (var i = 0; i < reqList.Count; i++)
            {
                var req = reqList[i];
                var paths = useMip
                    ? mipPaths[i] ?? SolvePricingMipFor(req, warmStartMip ? heuristicPaths[i].FirstOrDefault() : null)
                    : heuristicPaths[i];

                var maxCost = CombinedConvexDual(req.Id);
                var tol = 1e-8 + (double.IsPositiveInfinity(maxCost)
                    ? 0d
                    : Math.Abs(maxCost) * toleranceFraction);

                result[req.Id] = paths
                    .Where(p => p.TotalCost < maxCost - tol)
                    .OrderBy(p => p.TotalCost)
                    .Take(n)
                    .ToList();
            }

            return result;

            List<ShortestPath> SolvePricingMipFor(Request req, ShortestPath? warmStart)
            {
                var (arcWeight, resourceWeight) = PricingWeights(req);
                return SolvePricingMip(_env, req, n, arcWeight, resourceWeight, subResources[req.Id], forbiddenArcs, warmStart);
            }

            (Func<Arc, double> ArcWeight, Func<string, double> ResourceWeight) PricingWeights(Request req)
            {
                var vol = req.DemandGbps;

                double ArcWeight(Arc arc)
                {
                    var pi = CombinedCapDual(arc.Id);
                    var isDummy = arc.LinkId.StartsWith("dummy[", StringComparison.Ordinal);
                    var arcCost = (includeArcCost || isDummy)
                        ? arc.UsageCostPerGbps * vol
                        : 0d;
                    return arcCost - pi * vol;
                }

                double ResourceWeight(string resId)
                {
                    if (_mode != CGMode.LegRequest)
                        return 0d;
                    var absentWeight = resAbsentWeight.TryGetValue(resId, out var aw) ? aw : 0d;
                    return -CombinedResReqDual((req.Id, resId), _network.GetResource(resId), absentWeight);
                }

                return (ArcWeight, ResourceWeight);
            }
        }

        // Builds and solves the pricing MIP (6a)-(6b) for one request and returns
        // the distinct o_r -> d_r paths found in the solution pool.
        private List<ShortestPath> SolvePricingMip(
            GRBEnv env,
            Request req,
            int n,
            Func<Arc, double> arcWeight,
            Func<string, double> resourceWeight,
            HashSet<string> resources,
            IReadOnlySet<string>? forbiddenArcs,
            ShortestPath? warmStart = null)
        {
            var sub = req.SubNetwork!.Network;
            using var model = new GRBModel(env) { ModelName = $"Pricing[{req.Id}]" };
            model.Parameters.OutputFlag = 0;
            model.Parameters.Threads = 1;
            if (n > 1)
            {
                model.Parameters.PoolSolutions = n;
                model.Parameters.PoolSearchMode = 2;
            }

            var x = new Dictionary<string, GRBVar>(StringComparer.Ordinal);
            var arcs = new Dictionary<string, Arc>(StringComparer.Ordinal);
            foreach (var arc in sub.Arcs)
            {
                var ub = forbiddenArcs is not null && forbiddenArcs.Contains(arc.Id) ? 0d : 1d;
                x[arc.Id] = model.AddVar(0d, ub, arcWeight(arc), GRB.BINARY, $"x[{arc.Id}]");
                arcs[arc.Id] = arc;
            }

            var w = new Dictionary<string, GRBVar>(StringComparer.Ordinal);
            foreach (var resId in resources)
                w[resId] = model.AddVar(0d, 1d, resourceWeight(resId), GRB.BINARY, $"w[{resId}]");

            // (6a) flow conservation: in - out = -1 at o_r, +1 at d_r, 0 otherwise.
            foreach (var node in sub.Nodes)
            {
                var expr = new GRBLinExpr();
                foreach (var arc in sub.IncomingTo(node))
                    expr.AddTerm(1d, x[arc.Id]);
                foreach (var arc in sub.OutgoingFrom(node))
                    expr.AddTerm(-1d, x[arc.Id]);
                var rhs = node == req.Source ? -1d : node == req.Target ? 1d : 0d;
                model.AddConstr(expr == rhs, $"flow[{node}]");
            }

            // (6b) x_a <= w_{s(a)}.
            foreach (var arc in sub.Arcs)
            {
                if (arc.ResourceId is null || !w.TryGetValue(arc.ResourceId, out var wVar))
                    continue;
                var lhs = new GRBLinExpr();
                lhs.AddTerm(1d, x[arc.Id]);
                var rhs = new GRBLinExpr();
                rhs.AddTerm(1d, wVar);
                model.AddConstr(lhs <= rhs, $"link[{arc.Id}]");
            }

            // MIP start from the SP-CSS heuristic path (x_a = 1 on its arcs, w_s = 1
            // on its resources, 0 elsewhere).
            if (warmStart is not null)
            {
                var startArcs = new HashSet<string>(warmStart.Arcs.Select(a => a.Id), StringComparer.Ordinal);
                var startRes = new HashSet<string>(StringComparer.Ordinal);
                foreach (var a in warmStart.Arcs)
                    if (a.ResourceId is not null)
                        startRes.Add(a.ResourceId);
                foreach (var (arcId, xVar) in x)
                    xVar.Start = startArcs.Contains(arcId) ? 1d : 0d;
                foreach (var (resId, wVar) in w)
                    wVar.Start = startRes.Contains(resId) ? 1d : 0d;
            }

            model.ModelSense = GRB.MINIMIZE;
            model.Optimize();

            var found = new List<ShortestPath>();
            if (model.SolCount == 0)
                return found;

            var signatures = new HashSet<string>(StringComparer.Ordinal);
            var solCount = Math.Min(model.SolCount, n);
            for (var k = 0; k < solCount; k++)
            {
                model.Parameters.SolutionNumber = k;
                var used = new List<Arc>();
                foreach (var (arcId, xVar) in x)
                    if (xVar.Xn > 0.5)
                        used.Add(arcs[arcId]);

                var pathArcs = ExtractPath(used, req.Source, req.Target);
                if (pathArcs is null)
                    continue;
                if (!signatures.Add(string.Join("|", pathArcs.Select(a => a.Id))))
                    continue;

                // Cost of the extracted simple path (any zero-cost cycle in the MIP
                // solution is dropped): arc terms plus each touched resource once.
                var cost = 0d;
                var seen = new HashSet<string>(StringComparer.Ordinal);
                foreach (var arc in pathArcs)
                {
                    cost += arcWeight(arc);
                    if (arc.ResourceId is not null && resources.Contains(arc.ResourceId) && seen.Add(arc.ResourceId))
                        cost += resourceWeight(arc.ResourceId);
                }

                var nodes = new List<string> { req.Source };
                nodes.AddRange(pathArcs.Select(a => a.Target));
                found.Add(new ShortestPath(nodes, pathArcs, cost));
            }

            return found;
        }

        // Cached CSS structure per request. The arcs of each CSS only depend on
        // the resource chains, the request's sub-network and the forbidden-arc
        // set, so they are built once and rebuilt only when the forbidden set
        // (branching) changes. Costs are re-evaluated from the duals on each call.
        private sealed record CssEdge(string From, string To, Arc[] Arcs, string? ResourceId);
        // Concurrent because heuristic pricing runs requests in parallel.
        private readonly ConcurrentDictionary<string, (IReadOnlySet<string>? Forbidden, List<CssEdge> Edges)> _cssCache = new(StringComparer.Ordinal);

        private List<CssEdge> GetCssEdges(Request req, HashSet<string> resources, IReadOnlySet<string>? forbiddenArcs)
        {
            if (_cssCache.TryGetValue(req.Id, out var cached) && SameForbidden(cached.Forbidden, forbiddenArcs))
                return cached.Edges;

            var subArcs = new Dictionary<string, Arc>(StringComparer.Ordinal);
            foreach (var arc in req.SubNetwork!.Network.Arcs)
                if (forbiddenArcs is null || !forbiddenArcs.Contains(arc.Id))
                    subArcs[arc.Id] = arc;

            var edges = new List<CssEdge>();
            foreach (var arc in subArcs.Values)
                if (arc.ResourceId is null || !resources.Contains(arc.ResourceId))
                    edges.Add(new CssEdge(arc.Source, arc.Target, new[] { arc }, null));

            foreach (var resId in resources)
            {
                var res = _network.GetResource(resId);
                if (!res.IsChain)
                {
                    foreach (var ra in res.Arcs)
                        if (subArcs.TryGetValue(ra.Id, out var a))
                            edges.Add(new CssEdge(a.Source, a.Target, new[] { a }, resId));
                    continue;
                }

                var chain = res.Arcs;
                for (var k = 0; k < chain.Count; k++)
                {
                    var seq = new List<Arc>();
                    for (var k2 = k; k2 < chain.Count; k2++)
                    {
                        if (!subArcs.TryGetValue(chain[k2].Id, out var a))
                            break; // sub-sequence must be contiguous inside the sub-network
                        seq.Add(a);
                        edges.Add(new CssEdge(seq[0].Source, a.Target, seq.ToArray(), resId));
                    }
                }
            }

            IReadOnlySet<string>? snapshot = forbiddenArcs is null ? null : new HashSet<string>(forbiddenArcs, StringComparer.Ordinal);
            _cssCache[req.Id] = (snapshot, edges);
            return edges;
        }

        private static bool SameForbidden(IReadOnlySet<string>? a, IReadOnlySet<string>? b)
        {
            var ca = a?.Count ?? 0;
            var cb = b?.Count ?? 0;
            if (ca != cb)
                return false;
            if (ca == 0)
                return true;
            return a!.SetEquals(b!);
        }

        // Bellman-Ford pricing directly on the request's original arcs. Resource
        // duals are included when evaluating the returned path, once per resource.
        private List<ShortestPath> SolvePricingOriginalNetwork(
            Request req,
            Func<Arc, double> arcWeight,
            Func<string, double> resourceWeight,
            HashSet<string> resources,
            IReadOnlySet<string>? forbiddenArcs)
        {
            var sub = req.SubNetwork!.Network;
            var arcs = sub.Arcs
                .Where(a => forbiddenArcs is null || !forbiddenArcs.Contains(a.Id))
                .ToList();

            var dist = new Dictionary<string, double>(StringComparer.Ordinal) { [req.Source] = 0d };
            var pred = new Dictionary<string, Arc>(StringComparer.Ordinal);
            for (var it = 1; it < sub.Nodes.Count; it++)
            {
                var changed = false;
                foreach (var arc in arcs)
                {
                    if (!dist.TryGetValue(arc.Source, out var fromCost))
                        continue;
                    var candidate = fromCost + arcWeight(arc);
                    if (!dist.TryGetValue(arc.Target, out var targetCost) || candidate < targetCost - 1e-12)
                    {
                        dist[arc.Target] = candidate;
                        pred[arc.Target] = arc;
                        changed = true;
                    }
                }
                if (!changed)
                    break;
            }

            var found = new List<ShortestPath>();
            if (!dist.ContainsKey(req.Target))
                return found;

            var pathArcs = new List<Arc>();
            var visited = new HashSet<string>(StringComparer.Ordinal) { req.Target };
            var node = req.Target;
            while (node != req.Source)
            {
                if (!pred.TryGetValue(node, out var arc))
                    return found;
                pathArcs.Add(arc);
                node = arc.Source;
                if (!visited.Add(node))
                    return found;
            }
            pathArcs.Reverse();

            var total = 0d;
            var seenResources = new HashSet<string>(StringComparer.Ordinal);
            foreach (var arc in pathArcs)
            {
                total += arcWeight(arc);
                if (arc.ResourceId is not null && resources.Contains(arc.ResourceId) &&
                    seenResources.Add(arc.ResourceId))
                    total += resourceWeight(arc.ResourceId);
            }

            var nodes = new List<string> { req.Source };
            nodes.AddRange(pathArcs.Select(a => a.Target));
            found.Add(new ShortestPath(nodes, pathArcs, total));
            return found;
        }

        // SP-CSS heuristic pricing: every resource s = (a_1 -> ... -> a_|s|) is
        // replaced by its contiguous sub-sequences C(s) = {a_k -> ... -> a_k'},
        // each acting as a single arc with cost (Eq. 15)
        //     alpha'_{css,r} = sum_{a in css} (theta_ar - rho_a * v_r) - mu_{r,s}.
        // Arcs without a resource keep their own weight. A shortest path is then
        // computed with label-correcting (Bellman-Ford) since costs may be
        // negative. Returns at most one path; its cost is re-evaluated exactly as
        // in the MIP (each touched resource charged once).
        private List<ShortestPath> SolvePricingCss(
            Request req,
            Func<Arc, double> arcWeight,
            Func<string, double> resourceWeight,
            HashSet<string> resources,
            IReadOnlySet<string>? forbiddenArcs)
        {
            var sub = req.SubNetwork!.Network;
            var css = GetCssEdges(req, resources, forbiddenArcs);

            // Only costs are dual-dependent: evaluate them on the cached structure.
            var edges = new List<(string From, string To, double Cost, Arc[] Arcs)>(css.Count);
            var resWeights = new Dictionary<string, double>(StringComparer.Ordinal);
            foreach (var edge in css)
            {
                var cost = 0d;
                foreach (var a in edge.Arcs)
                    cost += arcWeight(a);
                if (edge.ResourceId is not null)
                {
                    if (!resWeights.TryGetValue(edge.ResourceId, out var rw))
                    {
                        rw = resourceWeight(edge.ResourceId);
                        resWeights[edge.ResourceId] = rw;
                    }
                    cost += rw;
                }
                edges.Add((edge.From, edge.To, cost, edge.Arcs));
            }

            // Bellman-Ford from the request source.
            var dist = new Dictionary<string, double>(StringComparer.Ordinal) { [req.Source] = 0d };
            var pred = new Dictionary<string, int>(StringComparer.Ordinal);
            var nodeCount = sub.Nodes.Count();
            for (var it = 0; it < nodeCount; it++)
            {
                var changed = false;
                for (var e = 0; e < edges.Count; e++)
                {
                    var (from, to, c, _) = edges[e];
                    if (!dist.TryGetValue(from, out var df))
                        continue;
                    var nd = df + c;
                    if (!dist.TryGetValue(to, out var dt) || nd < dt - 1e-12)
                    {
                        dist[to] = nd;
                        pred[to] = e;
                        changed = true;
                    }
                }
                if (!changed)
                    break;
            }

            var found = new List<ShortestPath>();
            if (!dist.ContainsKey(req.Target))
                return found;

            // Walk predecessors back; abort on a cycle.
            var segments = new List<Arc[]>();
            var visited = new HashSet<string>(StringComparer.Ordinal) { req.Target };
            var node = req.Target;
            while (node != req.Source)
            {
                if (!pred.TryGetValue(node, out var e))
                    return found;
                segments.Add(edges[e].Arcs);
                node = edges[e].From;
                if (!visited.Add(node))
                    return found;
            }
            segments.Reverse();
            var pathArcs = segments.SelectMany(s => s).ToList();

            // Reject paths that revisit a node through CSS internals.
            var pathNodes = new HashSet<string>(StringComparer.Ordinal) { req.Source };
            foreach (var a in pathArcs)
                if (!pathNodes.Add(a.Target))
                    return found;

            var total = 0d;
            var seen = new HashSet<string>(StringComparer.Ordinal);
            foreach (var a in pathArcs)
            {
                total += arcWeight(a);
                if (a.ResourceId is not null && resources.Contains(a.ResourceId) && seen.Add(a.ResourceId))
                    total += resourceWeight(a.ResourceId);
            }

            var nodes = new List<string> { req.Source };
            nodes.AddRange(pathArcs.Select(a => a.Target));
            found.Add(new ShortestPath(nodes, pathArcs, total));
            return found;
        }

        // Depth-first walk from source to target over the arcs selected by the MIP,
        // returning a simple path (cycles in the selection are skipped).
        private static List<Arc>? ExtractPath(List<Arc> used, string source, string target)
        {
            var outgoing = used.GroupBy(a => a.Source, StringComparer.Ordinal)
                               .ToDictionary(g => g.Key, g => g.ToList(), StringComparer.Ordinal);
            var visited = new HashSet<string>(StringComparer.Ordinal) { source };
            var stack = new List<Arc>();

            bool Dfs(string node)
            {
                if (node == target)
                    return true;
                if (!outgoing.TryGetValue(node, out var outs))
                    return false;
                foreach (var arc in outs)
                {
                    if (!visited.Add(arc.Target))
                        continue;
                    stack.Add(arc);
                    if (Dfs(arc.Target))
                        return true;
                    stack.RemoveAt(stack.Count - 1);
                }
                return false;
            }

            return Dfs(source) ? stack : null;
        }
    }
}
