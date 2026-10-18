# Jake workspace operating rules

## Durable long-running jobs — absolute rule

- Any job expected to outlive the current Codex turn or survive an app, terminal, SSH, browser, or laptop disconnect **must** run under a connection-independent supervisor such as `tmux` or a proper service manager.
- Never use a Codex tool session, foreground shell, ordinary background `&`, or `nohup` as the durability boundary for training, data generation, caching, rendering, evaluation, downloads, or chained experiment jobs.
- Put the complete dependent workflow—such as train → checkpoint verification → MSE/offline evaluation → closed-loop evaluation—inside the durable supervisor. Do not rely on a later Codex turn to start the next stage.
- A launch is not complete until all of the following are verified:
  1. The durable session or service exists.
  2. The real worker child PID is alive, not only the launcher shell.
  3. Logs or atomic status files continue updating after the launch command returns.
  4. The session name, reconnect command, PID, log path, status path, output path, and exact config are recorded.
  5. Resume/restart behavior is tested or explicitly defined, uses the latest verified checkpoint, and preserves prior logs and outputs.
- Never tell Jake that it is safe to close the Codex app, terminal, browser, or laptop until those checks pass.
- Monitoring must fail closed: stale status, missing checkpoints, incomplete metrics, or a dead worker count as failure, not success.
- On reconnect, inspect the durable session, child process, log freshness, GPU utilization, disk space, checkpoint integrity, and downstream-stage state before reporting that the job is healthy.

## Data generation model preference

- For future Codex-based robot training-data generation, prefer `gpt-6.1-sol` over `gpt-6-sol` when it is available to the active account/client and its applicable usage rate remains no higher. Check current model access and rates before each new production run; keep the existing quality gates and record the selected model in the run manifest.
