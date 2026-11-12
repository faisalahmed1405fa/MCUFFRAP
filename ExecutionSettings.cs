using Gurobi;

namespace MCFStatic;

internal static class ExecutionSettings
{
    public static bool SingleThreaded { get; set; }

    // Gurobi Method used by the CG restricted master LP.
    public static int LpMethod { get; set; } = 0;

    // Gurobi Method used by the weak LP relaxations (without and with SI cuts).
    public static int WeakLpMethod { get; set; } = -1;

    public static ParallelOptions ParallelOptions => new()
    {
        MaxDegreeOfParallelism = SingleThreaded ? 1 : -1
    };

    // 0 = sequential pricing MIPs; k > 0 = up to k concurrent pricing MIPs,
    // each in its own Gurobi environment (one license session per worker).
    public static int PricingMipWorkers { get; set; } = 4;

    public static void ConfigureGurobiEnvironment(GRBEnv env)
    {
        ArgumentNullException.ThrowIfNull(env);

        if (SingleThreaded)
            env.Set(GRB.IntParam.Threads, 1);
    }

    // Started, silent environment using the GUROBI_* license variables.
    public static GRBEnv CreateGurobiEnvironment(int? threads = null)
    {
        var env = new GRBEnv(empty: true);
        try
        {
            var licenseId = Environment.GetEnvironmentVariable("GUROBI_LICENSEID");
            var accessId = Environment.GetEnvironmentVariable("GUROBI_WLSACCESSID");
            var secret = Environment.GetEnvironmentVariable("GUROBI_WLSSECRET");
            if (!string.IsNullOrEmpty(licenseId))
                env.Set("LICENSEID", licenseId);
            if (!string.IsNullOrEmpty(accessId))
                env.Set("WLSACCESSID", accessId);
            if (!string.IsNullOrEmpty(secret))
                env.Set("WLSSECRET", secret);
            ConfigureGurobiEnvironment(env);
            if (threads is int t)
                env.Set(GRB.IntParam.Threads, t);
            env.Set(GRB.IntParam.OutputFlag, 0);
            env.Start();
            return env;
        }
        catch
        {
            env.Dispose();
            throw;
        }
    }
}
