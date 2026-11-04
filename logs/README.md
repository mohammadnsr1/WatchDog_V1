# logs

Run logs for every WatchDog session: the scored batches behind the paper's tables
and the unscored media runs behind the website videos. One directory per session,
named by its UTC start time. `index.csv` lists every session with its batch,
scenario and both agents' models.

## Layout

```text
logs/
  index.csv                         one row per session
  sessions/<session>/
    transcript_watchdog.jsonl.gz    WatchDog agent's Claude Code transcript (not in S4)
    transcript_executor.jsonl.gz    executor agent's Claude Code transcript
    agent_config_<role>.json        model, effort, CLI version and launch time per agent
    mcp_calls_<role>.jsonl          every MCP tool call each agent made, with its result
    events_<server>.jsonl           events from the motion, gripper, scene, watchdog and S4 monitor servers
    ground_truth_injections.jsonl   objects injected into the scene, with times and poses
    orchestrator/                   session start, step boundaries and run end
    authored_monitors/              monitors the WatchDog wrote itself (S2, S3)
    score.json, score_s4.json       conformance score (scored batches)
    observation_score.json          observation score (scored batches)
```

Not every file exists in every session; the set depends on the scenario.
Camera frames and ROS bags are not included.

## index.csv

| column | meaning |
|---|---|
| `batch` | the `paper_results/batches/<batch>/batch.yaml` that lists the session; empty for media runs |
| `purpose` | `scored`, `invalid` (listed under `known_invalid`, excluded from the tables), or `media` |
| `media` | `True` for the sessions behind the website videos |
| `scenario`, `assembly` | from `agent_config_*.json` |
| `<role>_model`, `<role>_effort` | Claude model and effort each agent ran at |
| `<role>_transcript` | path of the transcript file |

## Transcripts

Each transcript is the agent's Claude Code session file, one JSON record per line,
gzipped. Read one with `zcat transcript_watchdog.jsonl.gz | jq .`. Agent messages,
thinking, tool calls and tool results, including the images returned by `look()`,
are unchanged.

Before publishing, these were removed or redacted:

- the attachments that carry the operator's account and local context (`session_context`,
  `credential_org`), the operator's notes (`instructions`), and Claude Code's own
  system prompt (`prompt_snapshot`);
- the operator's email address, and the home directory, which is written as `~`.

In 24 S1 sessions the operator's auto-memory notes reached the agents through the
`instructions` attachment, before `autoMemoryEnabled` was turned off for S2 onward;
that attachment is removed here and the effect is recorded in
`paper_results/batches/S1/batch.yaml`.

## Regenerating

```bash
python3 -m analysis.export_logs            # rewrites logs/sessions and logs/index.csv
```

It reads `agents/transcripts/sessions/` and `~/.claude/projects/<repo slug>/`, matches
each agent's transcript by launch time (`analysis/transcripts.py`), and fails if any
output still contains a redacted string.
