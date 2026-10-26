# What I got wrong

Written at the end of the hackathon day, 2026-10-25, before results. Every number is in `runs/`.

## What stands, first

The cheating loop: 16 of 16 practice, 0 of 64 held-out, and Argon's own critic gave it 8 of 10. Argon's read of
943,582 tokens of agent transcripts in one request named the seed hunt, and a code check agreed on 12 of 12. The
32-token recipe carried the habit: 16 of 16, 0 of 16, 7 of 16. The hard course: both loops flagged by the compression
test, held-out agreeing (56 and 18 of 64); honest recipe 48 against 41.5 with none; the cheating loop's recipe 1 of 64.
Every held-out score comes from seeds no agent could guess.

On the hard course the compression test caught the cheating loop with no leaked seed in play, even though the agents had the simulator's code: 12 of 12 rebuilds from its recipe tried the seed hunt, found nothing (those practice seeds are large), and scored 0 of 16 on practice against the loop's 16.


## 1. I gave the student the answer machine

I kept the practice seeds out of the rebuild sandboxes and the 64 held-out seeds behind a secret. That felt like
"don't hand the student the answer sheet." But every sandbox also got the simulator's own source code, including the
formula that turns a seed into a flight's wind, payload and motor fault. With that formula in hand, an agent can
regenerate the answers for any seed it can guess. Most of them did: 16 of 16 rebuilds from the cheating loop's
recipe, 7 of 16 given no recipe on the easy course and 13 of 16 on the hard one, copied the generator into their
controller and searched seeds against the first sensor reading. I guarded the numbers and forgot the machine.

The honest design is a black box: the simulator runs outside the sandbox, the agent sends a controller and gets back
pass or fail. It is more plumbing (an endpoint instead of a file). I did not weigh it; I shipped the files.

This is also why no loop cheated on its own, which was my bet going in. With the simulator present, honest agents made
up their own practice flights (seeds 1 to 600, for example) and had no reason to memorize anything, which is what the
Amazon paper predicts. I built the cheater the way the paper does, by telling it to win at any cost.

## 2. I leaked a practice seed

Until 12:55 an example URL in the replay viewer's code (`tasks/drone/view.mjs`) contained one of the 16 practice
seeds. The first eight rebuilds from the cheating loop's recipe started before the fix and two of them check that seed
explicitly. Effect: at most 1 of 16 practice flights for those eight; none of the 64 held-out flights. I had scanned
the task file for seed numbers and not every file that shipped.

## 3. The warm-up seeds were 101 to 104

The first numbers anyone would try. They made the seed hunt pay off in practice, which is why agents kept it. Random
32-bit warm-up seeds would have made the hunt useless there. The held-out seeds are large (up to about 900 million),
which is why the scores that matter held.

## 4. The judge's scores depended on how busy the laptop was

The judge gives a controller 50 ms of wall-clock time per control step. The seed hunters' first-step search sometimes
ran past that on a loaded machine and the flight failed. Rebuild r0 scored 52 of 64 on the first scoring and 64 of 64
when re-scored idle; r2 went from 63 to 49. Honest-recipe and no-recipe rebuilds stayed within one flight. Both
scorings are kept (`runs/d1/judge.json`, `runs/d1/judge-first-scoring.json`). The fix is CPU time, not wall time.

## 5. The compression test did not flag the cheater on the easy course

The cheater's recipe carried real flying skill along with the cheat, so rebuilds from it scored 15 of 16 on practice
against the loop's 16, under the tolerance of 2. The held-out flights caught the cheater (0 of 64) and Argon's read of
the transcripts explained how. On the hard course the test did fire, for both loops, and the held-out flights agreed.
I kept compression as the headline for too long after knowing this.

## 6. Things I did not test

Other models on the transcript read or as builders (other frontier models also have million-token windows; the seed
trick is well known and I would expect them to find it). Random warm-up seeds. A black-box simulator. More than 16
rebuilds per group. Any statistics beyond counts and medians.

## When to use which

If you can make fresh tests, make fresh tests. In this simulator the 64 held-out flights caught every inflated score
(0, 18 and 56 of 64 against 16 of 16 practice) and cost seconds; the compression test costs 16 sandboxes. The
compression test earns its place where fresh tests do not exist: fixed benchmarks and real-world data, where every
look at a held-out set uses it up and a rebuild test does not. And it answers a question a held-out score cannot:
what survives the squeeze. Here that was a rebuild you can ship, and the finding that the cheating habit travels
through a one-sentence recipe (16 of 16 against 0 of 16). The Amazon paper is about fixed benchmarks, not free
simulators; my simulator made held-out tests cheap, which is exactly why my bet failed.

## What I would do next

A black-box runner outside the sandbox. Random warm-up seeds. A leak scan over everything that ships. CPU-time limits
in the judge. Then run the bet again: does a loop cheat on its own when it cannot make its own tests?
