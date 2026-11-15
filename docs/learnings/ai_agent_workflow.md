# Working with AI coding agents: what worked here

This repository was developed by one maintainer working with AI coding agents (a terminal agent plus parallel sub-agents). This page records the workflow and the failures, so you can adopt the parts that fit. Nothing here is specific to robotics.

## 1. The shape of the workflow

1. **Write the plan down before building.** A status document (`docs/ROADMAP.md`) lists what is done, partial and missing, with evidence for each line. It was produced first, reviewed by the maintainer, and is updated as items land. It is the contract between sessions: an agent that starts fresh reads it instead of guessing.
2. **Research before writing code.** Before building perception, navigation or coverage, agents surveyed existing open source and recent papers, and the result is the reuse map (section 17 of the roadmap). Several early conclusions were wrong (licences, version pins, a project that was unmaintained), and a second pass that read the primary sources (licence files, release pages) corrected them. Treat search-engine summaries as leads, not facts.
3. **Parallel agents with disjoint file ownership.** Independent pieces (housekeeping, docs, tool profiles, backend selection) ran as separate agents, each in its own git worktree, each told which files it owns and which it must not touch. Conflicts stayed small because the boundaries were written in the prompt.
4. **Stacked pull requests.** Work lands as a linear chain of small PRs, each based on the one below, instead of one large PR. Each is reviewable alone.
5. **Verify before merging.** A PR is gated on the full test suite from a clean checkout, a check for forbidden content (names, attribution lines, absolute paths), and a read-only review pass. Review findings become new commits on the PR's branch, are tested, and are pushed before the merge.

## 2. Rules that paid off

- **Never trust a report; check the artefact.** Agent summaries are written in a confident voice whether or not they are right. The checks that mattered: count the commits on the branch, grep the diff for forbidden strings, look at what a container actually mounts, read the log instead of the summary. Several "done" reports were really "written, not built".
- **Gate every commit and push on the test result.** A chained `test; commit; push` once pushed a failing state because the chain did not stop on failure. Use `&&`, or check the exit status.
- **Test from a clean checkout.** A full run from the agent's own working directory missed an install rule that referenced a directory that existed only on its disk (git does not track empty directories). A clean worktree caught it.
- **An instant "done" is a warning.** Two runs reported success in seconds because they tested an empty mount. If a ten-minute build finishes in two seconds, look at what ran.
- **Tell agents what base they start from.** Agent worktrees were created from the default branch, not from the stack, so agents silently lacked everything that was not merged. Every prompt began with the exact command that brings the base up to date.
- **Say what not to touch.** Prompts listed owned files and forbidden files, shared resources (images, volumes, ports, GPU) and cleanup duties. Unique `ROS_DOMAIN_ID` per container avoided cross-talk between parallel test runs.
- **Rules go in files, not in conversation.** Standing rules (commit identity, no attribution lines, no company names, no CI) live in project instructions and a memory directory that every session loads. A rule only said once in chat is lost at the next context reset.
- **Keep a persistent memory of decisions and corrections.** Short files, one fact each, with the reason and how to apply it. They record what the code cannot: why an approach was chosen, what the maintainer rejected, which environment quirks exist.
- **Prefer local computation to model reasoning.** Counting, grepping, diffing and arithmetic are done by running a command, not by asking the model to estimate. It is cheaper and exact.
- **Read primary sources for facts that gate decisions.** Licences from the LICENSE file, versions from the release page, behaviour from the code. Papers and blog posts are marked as such.

## 3. Mistakes and what they taught

| Mistake | Cost | Rule it produced |
|---|---|---|
| Pushed a fix while a test still failed | A red commit on a PR branch | Gate push on test status |
| Over-broad string replace while resolving a merge conflict changed a different block | A wrong plugin was loaded for one hardware mode | Add a test per selector; never replace on a substring that appears in differently indented code |
| Counted things from memory in the first draft of the roadmap | Several wrong numbers | Recount with a command, and date the figure |
| Too many parallel builds on one machine | Four builds shared the CPU and each ran several times slower | Cap concurrent builds; share a build cache; stagger the gate |
| Combined long commands hit the tool timeout | Lost output | Run long jobs in the background and wait on them separately |
| Network had no IPv4 route | `git push` and the GitHub CLI failed while other hosts worked | When one host fails and others work, check the address family before suspecting the tool |
| Searched for a skill that no longer existed | Time lost, then rewritten | Keep reusable skills in version control or a synced directory |

## 4. Testing policy that avoids waste

- A fix or feature that needs a test gets a test that fails without the change.
- Run the affected packages and their dependents while iterating; run the full suite once per PR before merge.
- Check for an existing test that covers the same behaviour before adding one. Prune duplicates and assertions that cannot fail.
- Keep slow simulator tests in their own tier. Quarantine flaky tests and fix them rather than rerunning.
- There is deliberately no CI service for this repository; the same checks run locally through `docker/build.sh`.

## 5. Reviewing with agents

- Reviews are read-only. A reviewer may not edit, commit or push.
- Each reviewer gets one dimension (correctness, concurrency, conventions, architecture, safety, tests, docs, supply chain) and must cite file and line.
- A second, adversarial pass tries to disprove each finding; unconfirmed findings are dropped or downgraded. This removed most false alarms.
- Findings are ranked, capped, and phrased as a failure scenario (inputs or state, wrong behaviour), not as taste.

## 6. When not to use an agent

A handful of tool calls, or a lookup whose target you already know, is faster done directly. Agents cost time to brief and attention to verify. They earn it on independent, well-bounded work with a clear exit test.
