using System.Diagnostics;
using System.Globalization;
using Gurobi;

namespace MCFStatic;

internal class Program
{
    static int Main(string[] args)
    {
        // Parse arguments and write CSV/log numbers with '.' decimals regardless of system locale.
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

        if (args.Length == 1 && args[0] is "--help" or "-h")
        {
            PrintUsage();
            return 0;
        }

        var parsedArgs = args;
        if (args.Any(argument => argument.StartsWith("--", StringComparison.Ordinal)))
        {
            if (!TryParseNamedArguments(args, out parsedArgs, out var parseError))
            {
                Console.Error.WriteLine(parseError);
                PrintUsage();
                return 1;
            }
        }
        else if (args.Length < 2)
        {
            PrintUsage();
            return 1;
        }

        var dataDir = parsedArgs[0];
        if (!Directory.Exists(dataDir))
        {
            Console.Error.WriteLine($"Data directory not found: {dataDir}");
            return 1;
        }

        NetworkKind kind;
        switch (parsedArgs[1].ToLowerInvariant())
        {
            case "static":
                kind = NetworkKind.Static;
                break;
            case "tele":
                kind = NetworkKind.Tele;
                break;
            default:
                Console.Error.WriteLine($"Invalid network kind '{parsedArgs[1]}'. Expected 'static' or 'tele'.");
                return 1;
        }

        CGMode cgMode = CGMode.LegRequest;
        if (parsedArgs.Length >= 3)
        {
            switch (parsedArgs[2].ToLowerInvariant())
            {
                case "leg":
                    cgMode = CGMode.Leg;
                    break;
                case "legrequest":
                    cgMode = CGMode.LegRequest;
                    break;
                default:
                    Console.Error.WriteLine($"Invalid CG mode '{parsedArgs[2]}'. Expected 'leg' or 'legrequest'.");
                    return 1;
            }
        }

        var arcMipTimeLimitSeconds = 600d;
        if (parsedArgs.Length >= 4)
        {
            if (!double.TryParse(parsedArgs[3], out arcMipTimeLimitSeconds) ||
                double.IsNaN(arcMipTimeLimitSeconds) || arcMipTimeLimitSeconds <= 0d)
            {
                Console.Error.WriteLine($"Invalid max runtime '{parsedArgs[3]}'. Expected a positive number of seconds.");
                return 1;
            }
        }
        if (!TryParsePositionalBoolean(parsedArgs, 4, "run-arc-mip", false, out var runmip) ||
            !TryParsePositionalBoolean(parsedArgs, 5, "run-cg", true, out var runcg))
        {
            return 1;
        }

        // Missing leg-request dual estimate policy: 0 = off, 1 = equal split over
        // all absent pairs on the arc, 2 = hybrid: 0 until the Lagrangian dual bound has
        // improved by less than hybrid-switch-tolerance over the last
        // hybrid-switch-stall-iters iterations, then 1. Policy 3 repeats 0 -> 1
        // on dual-bound stalls and 1 -> 0 on master-LP objective stalls. Policy 4 is
        // the reverse of 2: 1 until the master-LP objective stalls, then 0.
        var enable_dual_lb = 1;
        if (parsedArgs.Length >= 7)
        {
            switch (parsedArgs[6].ToLowerInvariant())
            {
                case "0":
                case "false":
                    enable_dual_lb = 0;
                    break;
                case "1":
                case "true":
                    enable_dual_lb = 1;
                    break;
                case "2":
                    enable_dual_lb = 2;
                    break;
                case "3":
                    enable_dual_lb = 3;
                    break;
                case "4":
                    enable_dual_lb = 4;
                    break;
                default:
                    Console.Error.WriteLine($"Invalid enable-dual-lb value '{parsedArgs[6]}'. Expected 0, 1, 2, 3, 4, true, or false.");
                    return 1;
            }
        }

        var integerMasterInterval = 50;
        if (parsedArgs.Length >= 8 &&
            (!int.TryParse(parsedArgs[7], out integerMasterInterval) || integerMasterInterval <= 0))
        {
            Console.Error.WriteLine($"Invalid CG integer-MIP interval '{parsedArgs[7]}'. Expected a positive integer.");
            return 1;
        }

        var phase2OptimalityTol = 0.0001;
        if (parsedArgs.Length >= 9 &&
            (!double.TryParse(parsedArgs[8], out phase2OptimalityTol) ||
             !double.IsFinite(phase2OptimalityTol) || phase2OptimalityTol <= 0d))
        {
            Console.Error.WriteLine($"Invalid CG optimality tolerance '{parsedArgs[8]}'. Expected a positive number.");
            return 1;
        }

        if (!TryParsePositionalBoolean(parsedArgs, 9, "branch-and-price", false, out var useBranchAndPrice))
            return 1;

        // Maximum depth of the branch-and-price dive.
        var branchDepth = 5;
        if (parsedArgs.Length >= 11 &&
            (!int.TryParse(parsedArgs[10], out branchDepth) || branchDepth < 0))
        {
            Console.Error.WriteLine($"Invalid branch-and-price depth '{parsedArgs[10]}'. Expected a non-negative integer.");
            return 1;
        }

        // Paths priced per request in each Phase 2 / branch-node pricing round.
        var cgNumPaths = 1;
        if (parsedArgs.Length >= 12 &&
            (!int.TryParse(parsedArgs[11], out cgNumPaths) || cgNumPaths <= 0))
        {
            Console.Error.WriteLine($"Invalid CG path count '{parsedArgs[11]}'. Expected a positive integer.");
            return 1;
        }

        // Run the weak LP relaxation and record its objective and run time.
        if (!TryParsePositionalBoolean(parsedArgs, 12, "run-weak-lp", false, out var runWeakLp))
            return 1;

        // Root-relaxation-only CG: run just the Phase 1 / Phase 2 LP root relaxations
        // (no integer master / integer-feasible solution) and report their run times.
        // Phase 1 completes when its LP objective is zero.
        if (!TryParsePositionalBoolean(parsedArgs, 13, "root-relaxation-only", true, out var rootRelaxationOnly) ||
            !TryParsePositionalBoolean(parsedArgs, 14, "single-threaded", false, out var singleThreaded))
        {
            return 1;
        }
        ExecutionSettings.SingleThreaded = singleThreaded;

        // Rounds a (request, arc) linking constraint may stay unused before it is
        // dropped from the LegRequest master; negative keeps it forever.
        var legReqIdleLimit = 0;
        if (parsedArgs.Length >= 16 && !int.TryParse(parsedArgs[15], out legReqIdleLimit))
        {
            Console.Error.WriteLine($"Invalid leg-request idle limit '{parsedArgs[15]}'. Expected an integer.");
            return 1;
        }

        // Maximum number of Phase 2 column generation iterations.
        var cgMaxIterations = 200;
        if (parsedArgs.Length >= 17 &&
            (!int.TryParse(parsedArgs[16], out cgMaxIterations) || cgMaxIterations <= 0))
        {
            Console.Error.WriteLine($"Invalid CG max iterations '{parsedArgs[16]}'. Expected a positive integer.");
            return 1;
        }

        var hybridSwitchTolerance = 0.01;
        if (parsedArgs.Length >= 18 &&
            (!double.TryParse(parsedArgs[17], out hybridSwitchTolerance) ||
             !double.IsFinite(hybridSwitchTolerance) || hybridSwitchTolerance <= 0d))
        {
            Console.Error.WriteLine($"Invalid hybrid switch tolerance '{parsedArgs[17]}'. Expected a finite positive fraction (0.01 = 1%).");
            return 1;
        }

        // Policies 2, 3 and 4: look-back window for relative bound improvement.
        // Policies 3 and 4 monitor master-LP decrease while policy 1 is active.
        var hybridSwitchStallIterations = 3;
        if (parsedArgs.Length >= 19 &&
            (!int.TryParse(parsedArgs[18], out hybridSwitchStallIterations) || hybridSwitchStallIterations <= 0))
        {
            Console.Error.WriteLine($"Invalid hybrid switch stall iterations '{parsedArgs[18]}'. Expected a positive integer.");
            return 1;
        }

        // Add the SI cuts x[a,r] <= y_{s(a)} to the weak LP relaxation; "both" solves
        // the weak LP without and then with the SI cuts in the same run.
        var weakLpVariants = new List<bool> { true };
        if (parsedArgs.Length >= 20)
        {
            switch (parsedArgs[19].ToLowerInvariant())
            {
                case "0":
                case "false":
                    weakLpVariants = new List<bool> { false };
                    break;
                case "1":
                case "true":
                    weakLpVariants = new List<bool> { true };
                    break;
                case "2":
                case "both":
                    weakLpVariants = new List<bool> { false, true };
                    break;
                default:
                    Console.Error.WriteLine($"Invalid weak-lp-si-cuts value '{parsedArgs[19]}'. Expected 0, 1, 2, true, false, or both.");
                    return 1;
            }
        }

        // Start CG pricing with the SP-CSS heuristic, then switch to the MIP.
        if (!TryParsePositionalBoolean(parsedArgs, 20, "cg-css-heuristic", true, out var cgCssHeuristic))
            return 1;
        Optimization.CssHeuristicEnabled = cgCssHeuristic;

        // Extra CG cleaning rounds a column may stay unused (X = 0, positive reduced
        // cost) before it is removed from the master; 0 removes it immediately,
        // negative never removes it. Values > 0 prevent remove/re-add cycling.
        var columnIdleLimit = 2;
        if (parsedArgs.Length >= 22 && !int.TryParse(parsedArgs[21], out columnIdleLimit))
        {
            Console.Error.WriteLine($"Invalid column idle limit '{parsedArgs[21]}'. Expected an integer.");
            return 1;
        }

        // Gurobi Method for the CG restricted master LP
        // (-1 auto, 0 primal, 1 dual, 2 barrier, 3 concurrent, 4 det. concurrent, 5 det. concurrent simplex).
        var lpMethod = 0;
        if (parsedArgs.Length >= 23 &&
            (!int.TryParse(parsedArgs[22], out lpMethod) || lpMethod < -1 || lpMethod > 5))
        {
            Console.Error.WriteLine($"Invalid LP method '{parsedArgs[22]}'. Expected an integer from -1 to 5.");
            return 1;
        }
        ExecutionSettings.LpMethod = lpMethod;

        // Gurobi Method for both weak LP relaxations (without and with SI cuts).
        var weakLpMethod = -1;
        if (parsedArgs.Length >= 24 &&
            (!int.TryParse(parsedArgs[23], out weakLpMethod) || weakLpMethod < -1 || weakLpMethod > 5))
        {
            Console.Error.WriteLine($"Invalid weak LP method '{parsedArgs[23]}'. Expected an integer from -1 to 5.");
            return 1;
        }
        ExecutionSettings.WeakLpMethod = weakLpMethod;

        // Parallel pricing MIP workers (0 = sequential, each worker owns a Gurobi env).
        var pricingMipWorkers = 4;
        if (parsedArgs.Length >= 25 &&
            (!int.TryParse(parsedArgs[24], out pricingMipWorkers) || pricingMipWorkers < 0))
        {
            Console.Error.WriteLine($"Invalid pricing MIP workers '{parsedArgs[24]}'. Expected a non-negative integer.");
            return 1;
        }
        ExecutionSettings.PricingMipWorkers = pricingMipWorkers;

        var arcsPath = Path.Combine(dataDir, "links.csv");
        var requestsPath = Path.Combine(dataDir, "requests.csv");

        // All run logs (CG phases + arc MIP) are written under <data-directory>/output.
        var outputDir = Path.Combine(dataDir, "output");
        Directory.CreateDirectory(outputDir);

        var loadSw = Stopwatch.StartNew();
        var network = DataIO.LoadNetwork(arcsPath, kind);
        var requests = DataIO.LoadRequests(requestsPath);
        loadSw.Stop();

        Console.WriteLine($"Loaded {kind} network: {network.NodeCount} nodes, {network.ArcCount} arcs.");
        var brokenResources = network.Resources.Count(s => !s.IsChain);
        Console.WriteLine($"Loaded {network.ResourceCount} resources (non-chain: {brokenResources}).");
        foreach (var s in network.Resources.Where(s => !s.IsChain))
            Console.WriteLine($"  Warning: resource {s} is not a single unbroken arc sequence.");
        var totalDemand = 0d; foreach (var r in requests.Values) totalDemand += r.DemandGbps;
        Console.WriteLine($"Loaded {requests.Count} requests (total demand {totalDemand:F2} Gbps).");
        Console.WriteLine($"Load time: {loadSw.Elapsed.TotalSeconds:F1} s");

        var solveSw = Stopwatch.StartNew();

        Parallel.ForEach(requests.Values, ExecutionSettings.ParallelOptions, request =>
        {
            request.SubNetwork = Optimization.FindFeasibleSubNetwork(network, request);
        });

        solveSw.Stop();

        var feasible = 0;
        var infeasible = 0;
        foreach (var request in requests.Values)
        {
            if (request.SubNetwork is not null)
                feasible++;
            else
                infeasible++;
        }

        Console.WriteLine($"Sub-network search completed in {solveSw.Elapsed.TotalSeconds:F1} s.");
        Console.WriteLine($"Feasible requests: {feasible}");
        Console.WriteLine($"Infeasible requests: {infeasible}");

        Console.WriteLine();
        Console.WriteLine("Per-request sub-network sizes:");
        foreach (var request in requests.Values)
        {
            if (request.SubNetwork is { } sub)
            {
                Console.WriteLine(
                    $"  {request.Id} ({request.Source} -> {request.Target}): {sub.Network.NodeCount} nodes, {sub.Network.ArcCount} arcs");
            }
            else
            {
                Console.WriteLine(
                    $"  {request.Id} ({request.Source} -> {request.Target}): infeasible");
            }
        }

        if (feasible == 0)
        {
            Console.WriteLine();
            Console.WriteLine("No feasible request to route; skipping arc-based optimization.");
            return 0;
        }
        
        
        IntegerMasterSolution? cgSolutionResult = null;
        var cgTimeToBestMs = double.NaN;
        var cgDualBound = double.NegativeInfinity;
        if (runcg)
        {
            Console.WriteLine();
            Console.WriteLine($"Running column-generation model (mode: {cgMode})...");

            try
            {
                var cgSw = Stopwatch.StartNew();
                Dictionary<string, double> branchingArcValues;
                BranchingArc? branchingCandidate;
                var cgSolution = useBranchAndPrice && !rootRelaxationOnly
                    ? Optimization.BranchAndPrice(network, requests, out cgTimeToBestMs, out cgDualBound, out branchingArcValues, out branchingCandidate, cgMode, maxIterations: cgMaxIterations, numPaths: cgNumPaths, dualLbPolicies: new[] { enable_dual_lb }, outputDir: outputDir, integerMasterInterval: integerMasterInterval, phase2OptimalityTol: phase2OptimalityTol, branchDepth: branchDepth, legReqIdleLimit: legReqIdleLimit, hybridSwitchTolerance: hybridSwitchTolerance, hybridSwitchStallIterations: hybridSwitchStallIterations, columnIdleLimit: columnIdleLimit)
                    : Optimization.CGoptimize(network, requests, out cgTimeToBestMs, out cgDualBound, out branchingArcValues, out branchingCandidate, cgMode, maxIterations: cgMaxIterations, numPaths: cgNumPaths, dualLbPolicies: new[] { enable_dual_lb }, outputDir: outputDir, integerMasterInterval: integerMasterInterval, phase2OptimalityTol: phase2OptimalityTol, rootRelaxationOnly: rootRelaxationOnly, legReqIdleLimit: legReqIdleLimit, hybridSwitchTolerance: hybridSwitchTolerance, hybridSwitchStallIterations: hybridSwitchStallIterations, columnIdleLimit: columnIdleLimit);
                cgSw.Stop();
                cgSolutionResult = cgSolution;

                if (cgSolution is null)
                {
                    Console.WriteLine($"Column generation did not find a feasible solution ({cgSw.Elapsed.TotalSeconds:F1} s).");
                    
                }
                else
                {
                    Console.WriteLine($"Column generation solved in {cgSw.Elapsed.TotalSeconds:F1} s.");
                    Console.WriteLine($"Objective: {cgSolution.Objective:F4}");

                    var cgRouted = 0;
                    foreach (var request in requests.Values)
                    {
                        if (cgSolution.SelectedPaths.TryGetValue(request.Id, out var pathIds) && pathIds.Count > 0)
                            cgRouted++;
                    }
                    Console.WriteLine($"Routed requests: {cgRouted}/{requests.Count}");

                    // Branching candidate from the final CG LP relaxation (for future
                    // branch-and-price): most fractional arc, tie-broken by highest cost.
                    if (branchingCandidate is not null)
                    {
                        Console.WriteLine(
                            $"Branching candidate: y[{branchingCandidate.ArcId}] = {branchingCandidate.Value:F4} " +
                            $"(|y-0.5| = {branchingCandidate.DistanceToHalf:F4}, cost = {branchingCandidate.Cost:F4})");
                    }
                    else
                    {
                        Console.WriteLine("Branching candidate: none (all arc variables integral).");
                    }
                }

                
            }
            catch (GRBException ex)
            {
                Console.Error.WriteLine($"Gurobi error ({ex.ErrorCode}): {ex.Message}");
                return 3;
            }
            
        }
        

        
        if (runmip)
        {
            Console.WriteLine();
            Console.WriteLine("Running arc-based MIP...");
            try
            {
                var arcMipLog = Path.Combine(outputDir, "arcmip.log");
                var cgTarget = cgSolutionResult?.Objective ?? double.NaN;
                var optSw = Stopwatch.StartNew();
                File.WriteAllText(arcMipLog, "Arc-based MIP starting" + Environment.NewLine);
                var solution = Optimization.SolveArcBasedModel(network, requests, out var arcLowerBound, out var arcTimeToMatchMs, targetObjective: cgTarget, logFile: arcMipLog, maxRunTimeSeconds: arcMipTimeLimitSeconds);
                optSw.Stop();

                if (solution is null)
                {
                    Console.WriteLine($"Arc-based MIP did not find an optimal solution ({optSw.Elapsed.TotalSeconds:F1} s).");
                    File.AppendAllText(arcMipLog,
                        $"{Environment.NewLine}Arc-based MIP did not find an optimal solution ({optSw.Elapsed.TotalSeconds:F1} s).{Environment.NewLine}");
                    return 2;
                }

                var mipRouted = 0;
                foreach (var request in requests.Values)
                {
                    if (solution.RequestPaths.TryGetValue(request.Id, out var arcs) && arcs.Count > 0)
                        mipRouted++;
                }

                var arcMipSummary =
                    $"Arc-based MIP solved in {optSw.Elapsed.TotalSeconds:F1} s.{Environment.NewLine}" +
                    $"Objective: {solution.Objective:F4}{Environment.NewLine}" +
                    $"Opened arcs: {solution.UsedArcs.Count}{Environment.NewLine}" +
                    $"Routed requests: {mipRouted}/{requests.Count}{Environment.NewLine}";

                Console.Write(arcMipSummary);
                File.AppendAllText(arcMipLog, Environment.NewLine + arcMipSummary);

                // Arc-based MIP results as CSV (single data row).
                var arcMipCsv =
                    $"solve_s,objective,opened_arcs,routed_requests,total_requests{Environment.NewLine}" +
                    $"{optSw.Elapsed.TotalSeconds:F1},{solution.Objective:F4},{solution.UsedArcs.Count},{mipRouted},{requests.Count}{Environment.NewLine}";
                File.WriteAllText(Path.Combine(outputDir, "arcmip.csv"), arcMipCsv);

                // Compare CG vs arc MIP: time-to-best, time-to-match, and the gap of the
                // CG incumbent against the best (largest) lower bound from either method.
                if (cgSolutionResult is not null)
                {
                    var bestLb = Math.Max(cgDualBound, arcLowerBound);
                    var cgInc = cgSolutionResult.Objective;
                    var gapPct = bestLb > 0 ? (cgInc - bestLb) / cgInc * 100d : double.NaN;
                    // Arc MIP never matched CG quality within the time limit: report ">runtime".
                    var arcMatchStr = double.IsNaN(arcTimeToMatchMs)
                        ? $">{optSw.Elapsed.TotalSeconds:F1}"
                        : arcTimeToMatchMs.ToString("F1");
                    var cmpCsv =
                        $"cg_incumbent,cg_time_to_best_s,arc_time_to_match_s,cg_dual_bound,arc_lower_bound,best_lower_bound,cg_gap_pct{Environment.NewLine}" +
                        $"{cgInc:F4},{cgTimeToBestMs:F1},{arcMatchStr},{cgDualBound:F4},{arcLowerBound:F4},{bestLb:F4},{gapPct:F4}{Environment.NewLine}";
                    File.WriteAllText(Path.Combine(outputDir, "comparison.csv"), cmpCsv);
                    Console.Write(cmpCsv);
                }
            }
            catch (GRBException ex)
            {
                Console.Error.WriteLine($"Gurobi error ({ex.ErrorCode}): {ex.Message}");
                return 3;
            }
            
        }
        

        if (runWeakLp)
        {
            // With a single variant the outputs keep the weaklp.* names; in "both" mode
            // the SI-cut run is written to weaklp_si.* next to the plain weaklp.*.
            var runBothWeakLps = weakLpVariants.Count > 1;
            foreach (var addSiCuts in weakLpVariants)
            {
                var label = addSiCuts ? "Weak LP relaxation with SI cuts" : "Weak LP relaxation";
                var fileStem = runBothWeakLps && addSiCuts ? "weaklp_si" : "weaklp";

                Console.WriteLine();
                Console.WriteLine($"Running {char.ToLowerInvariant(label[0])}{label[1..]}...");
                try
                {
                    var weakLpLog = Path.Combine(outputDir, fileStem + ".log");
                    File.WriteAllText(weakLpLog, label + " starting" + Environment.NewLine);

                    var weakSw = Stopwatch.StartNew();
                    var weakObjective = Optimization.WeakLPRelax(
                        network, requests, out _, out _,
                        logFile: weakLpLog, maxRunTimeSeconds: arcMipTimeLimitSeconds, addSiCuts: addSiCuts);
                    weakSw.Stop();

                    var weakSummary =
                        $"{label} solved in {weakSw.Elapsed.TotalSeconds:F1} s.{Environment.NewLine}" +
                        $"Objective: {weakObjective:F4}{Environment.NewLine}";
                    Console.Write(weakSummary);
                    File.AppendAllText(weakLpLog, Environment.NewLine + weakSummary);

                    // Weak LP relaxation results as CSV (single data row).
                    var weakLpCsv =
                        $"objective,solve_s{Environment.NewLine}" +
                        $"{weakObjective:F4},{weakSw.Elapsed.TotalSeconds:F1}{Environment.NewLine}";
                    File.WriteAllText(Path.Combine(outputDir, fileStem + ".csv"), weakLpCsv);
                }
                catch (GRBException ex)
                {
                    Console.Error.WriteLine($"Gurobi error ({ex.ErrorCode}): {ex.Message}");
                    return 3;
                }
            }
        }


       

        return 0;
    }

    private static void PrintUsage()
    {
        Console.Error.WriteLine("Usage: MCFStatic <data-directory> <static|tele> [leg|legrequest] [max-runtime-seconds] [run-arc-mip] [run-cg] [enable-dual-lb] [cg-integer-mip-interval] [cg-optimality-tol] [branch-and-price] [branch-depth] [cg-num-paths] [run-weak-lp] [root-relaxation-only] [single-threaded] [legreq-idle-limit] [cg-max-iterations] [hybrid-switch-tolerance] [hybrid-switch-stall-iters] [weak-lp-si-cuts] [cg-css-heuristic] [column-idle-limit] [lp-method] [weak-lp-method] [pricing-mip-workers]");
        Console.Error.WriteLine("   or: MCFStatic --data-directory <path> [--network-kind <static|tele>] [options]");
        Console.Error.WriteLine("  --network-kind <static|tele>                  default: static");
        Console.Error.WriteLine("  --cg-mode <leg|legrequest>                    default: legrequest");
        Console.Error.WriteLine("  --max-runtime-seconds <seconds>               default: 600");
        Console.Error.WriteLine("  --run-arc-mip <0|1|true|false>                default: false");
        Console.Error.WriteLine("  --run-cg <0|1|true|false>                     default: true");
        Console.Error.WriteLine("  --enable-dual-lb <0|1|2|3|4|true|false>       default: 1 (2: 0 then 1 on dual-bound stall; 3: alternate 0/1 on dual-bound/master-LP stalls; 4: 1 then 0 on master-LP stall)");
        Console.Error.WriteLine("  --cg-integer-mip-interval <positive integer>  default: 50");
        Console.Error.WriteLine("  --cg-optimality-tol <positive number>         default: 0.0001");
        Console.Error.WriteLine("  --branch-and-price <0|1|true|false>           default: false");
        Console.Error.WriteLine("  --branch-depth <non-negative integer>         default: 5");
        Console.Error.WriteLine("  --cg-num-paths <positive integer>             default: 1");
        Console.Error.WriteLine("  --run-weak-lp <0|1|true|false>                default: false");
        Console.Error.WriteLine("  --root-relaxation-only <0|1|true|false>       default: true");
        Console.Error.WriteLine("  --single-threaded <0|1|true|false>            default: false");
        Console.Error.WriteLine("  --legreq-idle-limit <integer>                 default: 0 (negative: never remove)");
        Console.Error.WriteLine("  --cg-max-iterations <positive integer>        default: 200");
        Console.Error.WriteLine("  --hybrid-switch-tolerance <positive fraction> default: 0.01 (1%; used by enable-dual-lb 2, 3 and 4)");
        Console.Error.WriteLine("  --hybrid-switch-stall-iters <positive integer> default: 3 (look-back window: 0 -> 1 on insufficient dual-bound increase; mode 3 also switches 1 -> 0 on insufficient master-LP decrease; resets after each mode-3 switch)");
        Console.Error.WriteLine("  --weak-lp-si-cuts <0|1|2|true|false|both>     default: true (x[a,r] <= y_s(a) in the weak LP; both/2: run without and with, SI results in weaklp_si.*)");
        Console.Error.WriteLine("  --cg-css-heuristic <0|1|true|false>           default: true (SP-CSS heuristic pricing before the MIP)");
        Console.Error.WriteLine("  --column-idle-limit <integer>                 default: 2 (extra rounds an unused column stays before removal; negative: never remove)");
        Console.Error.WriteLine("  --lp-method <-1..5>                           default: 0 (Gurobi Method for the CG restricted master LP)");
        Console.Error.WriteLine("  --weak-lp-method <-1..5>                      default: -1 (Gurobi Method for the weak LP and SI-cut LP)");
        Console.Error.WriteLine("  --pricing-mip-workers <non-negative integer>  default: 4 (parallel pricing MIPs, one Gurobi env each; 0 = sequential)");
        Console.Error.WriteLine("  Boolean options accept 0/1 or false/true. Do not mix named and positional arguments.");
    }

    private static bool TryParseNamedArguments(string[] args, out string[] parsedArgs, out string error)
    {
        parsedArgs = new string[] { "", "static", "legrequest", "600", "0", "1", "1", "50", "0.0001", "0", "5", "1", "0", "1", "0", "0", "200", "0.01", "3", "1", "1", "2", "0", "-1", "4" };
        error = "";
        var values = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

        for (var index = 0; index < args.Length; index++)
        {
            var argument = args[index];
            if (!argument.StartsWith("--", StringComparison.Ordinal))
            {
                error = $"Cannot mix named options with positional argument '{argument}'.";
                return false;
            }

            var option = argument[2..];
            var separator = option.IndexOf('=');
            string value;
            if (separator >= 0)
            {
                value = option[(separator + 1)..];
                option = option[..separator];
            }
            else if (index + 1 < args.Length && !args[index + 1].StartsWith("--", StringComparison.Ordinal))
            {
                value = args[++index];
            }
            else
            {
                error = $"Option '--{option}' requires a value.";
                return false;
            }

            if (!values.TryAdd(option, value))
            {
                error = $"Option '--{option}' was specified more than once.";
                return false;
            }
        }

        var optionIndexes = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase)
        {
            ["data-directory"] = 0,
            ["network-kind"] = 1,
            ["cg-mode"] = 2,
            ["max-runtime-seconds"] = 3,
            ["run-arc-mip"] = 4,
            ["run-cg"] = 5,
            ["enable-dual-lb"] = 6,
            ["cg-integer-mip-interval"] = 7,
            ["cg-optimality-tol"] = 8,
            ["branch-and-price"] = 9,
            ["branch-depth"] = 10,
            ["cg-num-paths"] = 11,
            ["run-weak-lp"] = 12,
            ["root-relaxation-only"] = 13,
            ["single-threaded"] = 14,
            ["legreq-idle-limit"] = 15,
            ["cg-max-iterations"] = 16,
            ["hybrid-switch-tolerance"] = 17,
            ["hybrid-switch-stall-iters"] = 18,
            ["weak-lp-si-cuts"] = 19,
            ["cg-css-heuristic"] = 20,
            ["column-idle-limit"] = 21,
            ["lp-method"] = 22,
            ["weak-lp-method"] = 23,
            ["pricing-mip-workers"] = 24
        };

        foreach (var (option, value) in values)
        {
            if (!optionIndexes.TryGetValue(option, out var valueIndex))
            {
                error = $"Unknown option '--{option}'.";
                return false;
            }

            if (valueIndex is 4 or 5 or 9 or 12 or 13 or 14)
            {
                if (!TryNormalizeBoolean(value, out var normalizedValue))
                {
                    error = $"Invalid value '{value}' for '--{option}'. Expected 0, 1, true, or false.";
                    return false;
                }

                parsedArgs[valueIndex] = normalizedValue;
            }
            else
            {
                parsedArgs[valueIndex] = value;
            }
        }

        if (!values.ContainsKey("data-directory"))
        {
            error = "Named syntax requires '--data-directory'.";
            return false;
        }

        return true;
    }

    private static bool TryParsePositionalBoolean(string[] parsedArgs, int index, string name, bool defaultValue, out bool result)
    {
        result = defaultValue;
        if (parsedArgs.Length <= index)
            return true;

        if (!TryNormalizeBoolean(parsedArgs[index], out var normalizedValue))
        {
            Console.Error.WriteLine($"Invalid {name} value '{parsedArgs[index]}'. Expected 0, 1, true, or false.");
            return false;
        }

        result = normalizedValue == "1";
        return true;
    }

    private static bool TryNormalizeBoolean(string value, out string normalizedValue)
    {
        switch (value.ToLowerInvariant())
        {
            case "0":
            case "false":
                normalizedValue = "0";
                return true;
            case "1":
            case "true":
                normalizedValue = "1";
                return true;
            default:
                normalizedValue = "";
                return false;
        }
    }
}
