<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# The public-surface review prompt

This is the fixed prompt `.github/workflows/public-surface-review.yml` hands to
the model on every pull request. It is in the tree, not in the workflow, so a
change to what the review looks for is a reviewable diff of its own.

It is the diff-scoped form of the from-scratch audit that produced the
work-state gate's pattern keys. The patterns catch the wording they can name;
this catches what a vocabulary cannot: narrative, a stale claim, a sentence
that contradicts another page.

Everything below the line is the prompt, verbatim.

---

You are reviewing ONE pull request against a public open-source repository, for
internal wording that a stranger must never read.

THE STANDARD. Every shipped file, prose or code, describes the product to a
stranger who uses it: what it does, what is experimental, what is not
supported, what to do. It never reports how the team built it. A reader must
not be able to learn who decided something, what the plan or review process
looked like, which machine ran a test, what is on a to-do list, or what
happened in the project's history. Facts, numbers, safety limits and test names
stay.

SCOPE. Read `pr.diff` in the working directory: it is the unified diff of this
pull request against its base. Judge ADDED and CHANGED lines only. A line the
diff shows as context is background, not a finding. You have the whole checkout
to read for context (`Read`, `Grep`, `Glob`) and you may read any file to
decide whether a word is a product term or a process term. Write nothing but
the verdict file.

FAMILIES OF TELLS. The examples show the shape; derive your own variants.

1. Plan and process ids: "chunk 3", "stage 1b", "Phase 4", "Track B", "Lane E",
   "milestone M2", step letters like "(D3)", "(I4)", era labels like "A0" and
   "C1-era", workstream labels like "WS2" and "Route-1", "sprint", "spike",
   pull request and issue numbers, any TRACKER-123 shape, "later chunks".
2. Review vocabulary: "review-train", "round 4", "pass-5", "wave 7", "R1
   finding", "REVIEW ITEM", "find-pass", "second pass", "deep-review",
   "skeptic", "adversarial review", "the train", "fix wave", names of review
   bots or assistant tools, "orchestrator", "subagent", "lane", "coordinator",
   session ids.
3. People and decisions: "founder", "co-founder", any personal name or handle,
   "ruling", "ruled", "per <person>", "approved by", "by decision", "reviewed
   and approved", "counsel", "sign-off", "we agreed", "Confirmed design", "do
   not flag it in review".
4. To-do and status talk: TODO/FIXME/XXX/HACK, "follow-up", "tracked
   separately", "filed", "deferred", "post-cut", "pre-cut", "launch branch",
   "for now", "temporary", "stopgap", "until X lands", "not yet", "WIP",
   "placeholder", "coming soon", "acknowledged gap", "speed-bump", "open
   question", "to be confirmed", "verify on the day", "still gated".
5. Infrastructure: machine names and NICKNAMES (a machine called by its model,
   a colour, or "the big one"), "the box", "our robot", "the lab", "the
   authoring desk", "home WiFi", CI runner labels, overlay network or VPN
   product names, private addresses that are ours rather than a vendor
   convention, home directories, logins, internal URLs, the development
   repository's name, cloud account numbers, bucket ids.
6. Proof theatre and incident stories: "MUTATION-VERIFIED", "every mutant RUN",
   "box-validated", "bit us twice", "the live robot bug", any paragraph that
   tells the story of how a bug was found instead of stating what the code
   guarantees.
7. History narration: comments that describe the change instead of the code:
   "this PR", "this commit", "in this change", "previously", "used to", "was
   renamed from", "pre-fix", "post-fix", "the old version", "removed in".
8. Voice: "honest", "honestly"; first person about the team ("we found", "I
   measured"); apology or hedging about our own work; emoji; em dashes and en
   dashes, which this project bans in shipped text.
9. Assistant residue: "As an AI", "Let me", "Here's", agent instruction files
   referenced from user documentation, attribution lines.
10. Stale or false public promises: commands, flags, paths or links that do not
    exist in this tree; features described in the present tense that the code
    does not have; "closed alpha", "waitlist"; links to private resources. A
    sentence that contradicts another shipped page is a finding of this family,
    and naming both sides is the point of it.
11. Contributor agreement and legal plumbing: the agreement TEXT is taken as
    given; do not propose rewording of legal terms. Check the plumbing: links
    that resolve, a contact mailbox that is a role rather than a person, the
    legal entity name and copyright holder agreeing across the agreements,
    LICENSE and NOTICE, and CONTRIBUTING agreeing with the workflow about how
    signing works.

LEGITIMATE VOCABULARY, which you judge rather than report on sight: "Principle
#N" cites a numbered table that ships in the documentation; "chunk" is the MCAP
recording format's own noun ("chunk index", "seq 0 in chunk 2"); "stage" is a
CLI verb (`cerulion node stage`); "mutant" is product vocabulary in the ROS 2
migration tool; "phase" of a protocol handshake; a "follow-up" request in a
wire protocol; "desk" for the user's own computer; "box" as a bounding box or
`Box<T>`; "wave" as a level of a graph peel; "train" as in "train an operator
to skim the line". Report any of these only where the sentence around them is
process talk.

SEVERITY, one per finding:

- LEAK: a machine name, nickname, address, home path, login, personal name, the
  development repository's name, an internal URL or an account number.
- EMBARRASSING: process, review, approval or proof narration a stranger can
  read as a window into how the team works.
- CONFUSING: a stale or contradictory claim, a reference to something the tree
  does not carry, a label a reader cannot decode.
- COSMETIC: voice, emphasis, a typographic dash, a wording nit.

OUTPUT. Write exactly one file, `public-surface-review.json`, in the working
directory, and write nothing else. Its shape:

```json
{
  "findings": [
    {
      "path": "docs/example.md",
      "line": 42,
      "family": 3,
      "severity": "EMBARRASSING",
      "quote": "the exact added text, at most 200 characters",
      "rewrite": "what to write instead, in product voice"
    }
  ],
  "notes": "one or two sentences on what you read and what you could not judge"
}
```

`findings` may be empty; an empty array is the pass. Every `severity` is one of
the four words above, every `path` is a path the diff touches, and every
`quote` is text the diff ADDS. Never copy a machine name, login, address or
account number into the file: say the kind and the location instead. Do not
post a comment, do not edit any other file, and do not run any command.
