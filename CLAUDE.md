# ProjectFusionBoard

## Superpowers Skills

This project uses superpowers skills from `.claude/skills/`. You MUST follow the `using-superpowers` protocol:

- Before ANY response or action, check if any superpowers skill might apply. Even a 1% chance means invoke it via the Skill tool.
- Process skills (brainstorming, systematic-debugging) take priority over implementation skills.
- Never skip a skill because the task seems "too simple" — that's a red flag. Simple tasks benefit most from structured process.

Key skills available:
- **brainstorming** — before any creative/feature work
- **writing-plans** — break designs into small implementation tasks
- **subagent-driven-development** — execute plans with subagent pipeline
- **executing-plans** — batch execute tasks
- **dispatching-parallel-agents** — parallel agent dispatch
- **test-driven-development** — RED-GREEN-REFACTOR cycle
- **systematic-debugging** — 4-stage root cause analysis
- **verification-before-completion** — verify fixes before claiming done
- **requesting-code-review** — self-review before requesting review
- **receiving-code-review** — process review feedback
- **using-git-worktrees** — isolate work in git worktrees
- **finishing-a-development-branch** — merge/PR decision flow
- **writing-skills** — create new custom skills
