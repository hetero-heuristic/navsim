# A Certified Lexicographic Benchmark for Driving Trajectories

*Design note. Working title for the scheme: **LexBench**; the per‑scene quantity is the
**Lexicographic Rulebook Score (LRS)**; its NAVSIM‑facing instantiation is **L‑EPDMS**
(lexicographic EPDMS). Names are placeholders.*

This note proposes a way to benchmark planned driving trajectories — the task NAVSIM's
EPDMS performs — using a lexicographic (rulebook) ordering rather than a weighted/
multiplicative scalar. It is written against three audits of your own code (LexiCert,
LexiCone, and the NAVSIM fork), and its central aim is to keep the part of the
lexicographic programme that the audits found *sound* while discharging, by construction,
the parts they found *broken*. The short version of the thesis:

> A benchmark **ranks already‑computed trajectories**; it does not optimise. The
> lexicographic comparison of two trajectories is therefore **exact and solver‑free**. In
> that setting the LexiCert certificate stops being an approximation with a tolerance band
> and becomes an *exact, floating‑point‑safe, trivially auditable* order‑preserving scalar.
> Everything the LexiCert audit charged against the online certificate (the inert `σ̂`
> instrument, the non‑exact "exact cascade", the 10⁻⁶ floor, the admissibility gate that
> never fires) simply does not arise when you are scoring rather than solving. What remains
> is a measurement‑discipline problem, and all three audits already tell us precisely how to
> get the measurement right.

---

## 1. What the audits establish (the design‑relevant distillation)

The three repositories are three facets of one programme — *replace a lexicographic cascade
of trajectory optimisations by something cheaper, and prove the replacement is faithful* —
and the audits cleanly separate what survived scrutiny from what did not.

**LexiCert — the theory is sound; the instruments and the closed loop are not.** The proof
chain (Hoffman error bound with explicit constant, the one‑step engine, the main theorem,
zero propagation, the ladder corollary, the polyhedral proposition) was checked by hand and
numerically and holds. For a *fixed* stacked constraint matrix with right‑hand‑side‑only
scene variation, one weighted LP solve is certified ε‑lexicographically‑minimal uniformly
over the class, with a closed‑form weight ladder `w_i = τ κ_i Λ_i`,
`Λ_i = ℓ_0 + Σ_{j>i} w_j ℓ_j`. What failed is everything *around* the theorem:

- the online second check `σ̂` is a basis identity that is numerically zero for any simplex
  basis, so it certifies nothing (F04); nothing gates on admissibility (F05);
- the per‑tick pass is really against the `1e-6` floor, not the certified band, which is
  decades tighter (F29); the "exact cascade" reference is itself pinned and biased two‑sided
  (F25, F27);
- the closed‑loop driver executes a *different* trajectory than the certified one — a Frenet
  velocity inverse drops the `(1−κd₀)` metric factor and produces a 3.5→31.9 m/s runaway
  under −6 m/s² braking (F01), the chord‑velocity interface executes exactly half of every
  planned acceleration (F08), and the yaw label swings a near‑standstill ego by up to 1.55
  rad per step (F11);
- several rule rows do not constrain what they claim (F12–F14, F45, F46);
- evidence discipline is weaker than advertised (altered frozen E4 records, 13 figures
  bypassing the freeze guard, every manifest `git_commit: unavailable`) (F03, F17, F18, F61);
- two "pre‑registered" fixture results were defined *after* failing runs (F06, F07).

**LexiCone — the scalarisation‑of‑lexicographic trap, stated as a theorem‑level
counterexample.** LexiCone tries to replace the cascade by *one calibrated weighted solve*
for a nuPlan MPC planner. The audit's F01 is the single most important fact for any
benchmark that wants to compress a lexicographic order into a scalar:

> Driving `w_1 → ∞` with the other weights fixed does **not** recover the full
> lexicographic trajectory. On `Z=[−1,1]²`, `V_1=[x]_+²`, `V_2=[y]_+²`,
> `J=½(x−1)²+½(y−1)²`, the lex optimum is `(0,0)` but the weighted optimum tends to
> `(0,1/3)`: one diverging weight enforces only the top level and leaves a finite
> trade‑off below.

The rest of LexiCone's math audit is the same lesson in different clothes: a Chebyshev
"radius in weight space" that is actually a lifted‑slack margin (F03), a pseudo‑inverse
multiplier that rejects feasible weights (F04), a "forced relaxation" that labels satisfiable
lower rules as violated (F05), and a general reminder (04 §3) that **within one tier a
weighted sum silently trades off rules that a strict order keeps separate.** The observer /
Construe side is the positive model: typed per‑rule results, qualified cross‑validation,
`NOT_APPLICABLE`/`UNSUPPORTED`/`INDETERMINATE` as first‑class outcomes, and a refusal to call
two predicates "the same rule" without a verified mapping.

**NAVSIM — EPDMS is coherent but its aggregation and several sub‑scores respond to the scene,
not the planner.** Confirmed against the code:

- EPDMS = `(∏_{m∈{NC,DAC,DDC,TLC}} filter_m) · (Σ_{m∈{EP,TTC,LK,HC,EC}} w_m filter_m / Σ w_m)`
  with weights `5,5,2,2,2`. The first block is a multiplicative gate; the second is a
  **weighted arithmetic mean** — a scalarisation, with every trade‑off pathology that
  implies.
- Progress (EP) is normalised by the *best proposal*, so when the PDM‑Closed reference fails
  a multiplier, every agent gets EP = 1 (F08). Lane‑keeping is distance to the PDM‑Closed
  centreline, so a legitimate long lane change scores 0 (F27/§10). At‑fault collision class
  is read from the track's *first‑seen* velocity (F06); a t=0 overlap zeros a multiplicative
  metric for all proposals (F09).
- The human‑penalty filter is binary and asymmetric: *any* human metric = 0 gives the agent
  full credit on that metric, and the filter is inert for EP/HC (F20).
- The two‑stage pseudo‑closed‑loop weighting is discontinuous (jumps to uniform at 1.92 m),
  keys off the *commanded* rather than the *simulated* endpoint, and one bad token aborts the
  whole run after all compute (F07, F10, F03).
- Reactive traffic never receives traffic‑light status, so IDM agents brake to a halt at the
  end of their first lane and ignore red lights, and they negotiate against an ego displaced
  1.46 m rearward (F01, F02 in the NAVSIM audit).

Both the LexiCert and LexiCone audits end with a **"recommended measurement design"** section
(LexiCert 04 §7, LexiCone 04 §9) and both say the same thing: *typed results with units,
shared gates between predicate and monitor, durations in seconds, absolute not
reference‑relative metrics, `violation_frames ≤ applicable_frames`, denominators beside every
aggregate, and positive/negative fixtures per rule.* Those sections are, in effect, the
specification for the measurement layer below.

---

## 2. Why a weighted/multiplicative scalar is the wrong aggregator

Two concrete EPDMS behaviours show the problem, and both are *correct* arithmetic — the
defect is the aggregation choice, not a bug.

**Cross‑objective compensation in the weighted block.** Take two trajectories that are equal
on every multiplier and on safety/legality, and compare only the quality block (use the
shipped `pdm_score` block `{EP,TTC,LK,HC}`, weights `5,5,2,2`, sum 14):

| | EP | TTC | LK | HC | weighted block |
|---|---:|---:|---:|---:|---:|
| A (aggressive) | 1.0 | 1 | 1 | 0 | (5+5+2+0)/14 = **0.857** |
| B (comfortable, slower) | 0.4 | 1 | 1 | 1 | (2+5+2+2)/14 = **0.786** |

EPDMS ranks A above B: it has *priced two units of comfort against `5·0.6 = 3` units of
progress*. If your intended priority is COMFORT ≻ PROGRESS (the usual driving order), the
lexicographic verdict is the opposite — B wins, because comfort is compared before progress
can speak. EPDMS cannot express "progress must never buy back a comfort deficit"; a
lexicographic order can, and makes the priority explicit instead of hiding it in `5` vs `2`.

**The multiplicative gate mis‑ranks graded failures.** `NC ∈ {0, ½, 1}`: a collision with a
static object scores ½, which *multiplies* the whole score by ½.

| | NC | rest (quality block) | EPDMS |
|---|---:|---:|---:|
| C (hits a static object, else perfect) | ½ | 1.0 | ½·1.0 = **0.50** |
| D (collision‑free, mediocre quality) | 1 | 0.40 | 1·0.40 = **0.40** |

EPDMS ranks the *static collision* above the clean‑but‑mediocre trajectory. "Safety first" is
not what a product of a ½ and a good quality score encodes. A lexicographic order puts any
safety violation — graded, so "static object" is a small positive violation rather than a
catastrophe — strictly ahead of every comfort/progress difference, so D wins.

**A scalar also hides ties and manufactures precision.** EPDMS returns one float; two
planners differing by `1e-15` are "ranked". LexiCert's own F51 is this disease inside the
planner: a 1‑ulp difference (`24.122889801085353` vs `24.12288980108535`) "decides" a tier.
A benchmark that wants to drive research must say *when two trajectories are indistinguishable
at the measurement's resolution*, not invent an order between them.

The remaining EPDMS defects (degenerate EP reference, reference‑relative LK, first‑seen
collision class, binary human filter, discontinuous two‑stage weighting) are all the same
underlying move: **the score responds to scene properties or to a reference, so it is not a
property of the trajectory.** A good benchmark score must be (i) a function of the trajectory
and the scene alone, (ii) absolute, (iii) graded, and (iv) honest about ties. Lexicographic
aggregation plus audit‑hardened measurement gives all four.

---

## 3. The scheme

### 3.1 A rulebook, not a weight vector (Pillar 1)

Define an explicit **priority pre‑order** of rule tiers. For driving the natural order is

```
T0 SAFETY  ≻  T1 LEGAL  ≻  T2 ROAD/DRIVABLE  ≻  T3 COMFORT  ≻  T4 PROGRESS/PERFORMANCE
```

This is Censi et al.'s *rulebook* (ICRA 2019): a pre‑order over rules together with a
per‑rule violation metric, inducing a hierarchical order over trajectories. Within a tier,
rules may be either *totally ordered* (split them into sub‑tiers) or *equally important*
(combine them by a stated within‑tier reduction). The combination inside a tier is a local
scalarisation and **must be declared as a trade‑off zone** — this is LexiCone 04 §3's first
distinction, and it is the one place the scheme tolerates compensation, deliberately and
visibly. Everything *across* tiers is strict.

The rulebook is the design artefact that replaces EPDMS's `{NC,DAC,DDC,TLC}` gate and
`5/5/2/2/2` weights. It is auditable, editable, and its priority decisions are explicit
rather than implied by magnitudes.

### 3.2 Graded, bounded, absolute, typed violation metrics (Pillar 2)

Each rule returns a **typed result** — the exact object both audits ask for
(LexiCert 04 §7, LexiCone 04 §9):

```
RuleResult = {
  outcome:      satisfied | violated | not_applicable | indeterminate,
  applicability: bool + gate parameters (in SI units),
  raw_value:    float with declared units,
  severity:     v ∈ [0, B_i]      # bounded, monotone, ABSOLUTE normalisation
  norm_version: str,              # which normalisation produced `severity`
  frame_grid:   dt, n_frames,
  evidence:     per-frame support
}
```

Two properties are mandatory and are where "much better than EPDMS" is earned:

- **Graded, not binary.** A collision contributes a severity that grows with penetration
  depth × relative speed × at‑fault factor and saturates at `B_0`; a red‑light infraction
  grows with speed‑past‑bar; progress is a *fraction of an absolute route reference*. Grading
  is what lets the benchmark rank *within* the failure set — the essence of minimum‑violation
  planning (Tumova et al.; Castro, Karaman, Frazzoli). EPDMS's binary gates collapse every
  failure to 0 and cannot tell "clipped a cone" from "drove through a crowd".
- **Absolute, not reference‑relative.** `severity` is a function of the trajectory and the
  scene only — never normalised by the best proposal (fixes EPDMS EP, F08) or measured as
  distance to the reference planner's path (fixes EPDMS LK, F27). A human/reference comparison
  may still be *reported as a diagnostic*, but it never enters the ranking.

### 3.3 Exact lexicographic comparison as ground truth (Pillar 3)

Quantise each tier severity to its **pre‑registered indifference resolution** `δ_i > 0`:

```
q_i(a) = round( v_i(a) / δ_i ) ∈ {0, 1, …, Q_i},   Q_i = ceil(B_i / δ_i).
```

Compare two trajectories by the lexicographic order on the integer tuples
`q(a) = (q_0, q_1, …, q_L, q_J)`. This is exact, needs no weights and no solver, and ties are
*exact* (equal quantised tuples), which is the principled fix to LexiCert F51: differences
below `δ_i` are declared indistinguishable rather than ranked. The tuple order is a genuine
total pre‑order; incomparability only appears across scenes (§3.5).

`δ_i` is a first‑class, pre‑registered parameter and it is exactly where the policy question
"how much safety difference is meaningful?" lives — out in the open, per tier, instead of
buried in EPDMS's weights and the ½‑collision multiplier. It is also the answer to the
standard objection to lexicographic scoring ("a `1e-6` safety difference dominates
everything"): sub‑`δ_0` safety differences are ties, so the next tier decides.

### 3.4 A certified order‑preserving scalar (Pillar 4) — the clean reuse of LexiCert

Leaderboards want one number. Encode the tuple order as

```
S(a) = Σ_{i} w_i · q_i(a),     w_i = Π_{j>i} (Q_j + 1),   (set w for the lowest tier = 1)
```

This is the **mixed‑radix (positional‑number) encoding** of the tuple, and it reproduces the
lexicographic order **exactly**: `S(a) < S(b) ⇔ q(a) <_lex q(b)`, with `S` injective on
tuples. The weight recursion `w_i = Π_{j>i}(Q_j+1)` is *exactly LexiCert's ladder*
`w_i = τ κ_i Λ_i`, `Λ_i = ℓ_0 + Σ_{j>i} w_j ℓ_j` — the separation condition "no accumulation
of lower tiers can bridge one higher‑tier step" `w_i > Σ_{j>i} w_j Q_j` — specialised to the
regime where

- the Lipschitz/Hoffman modulus is the **known integer bound** `Q_j` (metrics are bounded and
  quantised), not a Hoffman constant estimated by SVD, and
- the margin `τ` is the **exact `+1` integer gap** guaranteed by quantisation, not a
  `5%` cushion.

In that regime the certificate's ε‑band collapses to **exactly zero**. This is the payoff of
benchmarking rather than optimising, and it is worth stating plainly against the LexiCert
findings:

| LexiCert online certificate (optimising) | LexBench scalar (scoring) |
|---|---|
| `σ̂` must bound solver suboptimality — but is an inert basis identity (F04) | **No solve.** No `σ̂` at all. |
| Pass is against the `1e-6` floor, not the band (F29) | Band is **exactly 0**; agreement is bit‑exact. |
| "Exact cascade" reference is pinned, biased two‑sided (F25, F27) | The lexicographic tuple comparison **is** the exact reference, `O(L)` to compute. |
| Admissibility gate `w_i > Λ_i κ_i` never enforced (F05) | Separation holds **by construction** from known `Q_j`. |
| Weight span `w_1/w_4 = 10^{7.2}` strains conditioning | Span is `Π(Q_j+1)`; see the float budget below. |

**Float budget (state it, as LexiCert should have).** `S` is an exact integer iff
`Π_i (Q_i+1) ≤ 2^{53}`, i.e. `Σ_i log₂(Q_i + 1) ≤ 53`. With five tiers that is ~10.6 bits
(~1500 levels) per tier — comfortable. If a tier needs finer resolution than the budget
allows, you have three honest options, in order of preference: **keep the tuple** (no scalar
is actually required for ranking; sort lexicographically), use integer/bignum arithmetic, or
coarsen `δ_i`. Recommendation: **the tuple `q(a)` is primary; the scalar `S(a)` is a derived,
budget‑checked convenience** reported only when a single number is contractually needed
(e.g. a leaderboard column). This is strictly more honest than EPDMS forcing a float.

### 3.5 Honest cross‑scene aggregation (Pillar 5)

Averaging a per‑scene score across a corpus re‑introduces compensation *at the dataset level*
— a safe scene's high score offsets a catastrophic scene — which is the same disease EPDMS
has within a scene, and NAVSIM's two‑stage mean inherits it. Aggregate **tier by tier**
instead:

1. For each tier, report the corpus distribution: fraction of scenes with `q_i > 0`, the mean
   and worst severity *among applicable scenes*, with the **denominator and the
   not_applicable/indeterminate composition shown** (all three audits insist on this).
2. Rank planners by the **lexicographic aggregate**: compare first on the top tier's
   incidence/severity profile, then the next, so "safety first" holds at the corpus level and
   not only per scene.
3. Emit a single corpus scalar only as a derived, audited quantity, and *label it a deliberate
   scalarisation with the compensation caveat* — the discipline of the "what a score does /
   does not establish" sections the audits ship.

Treat `indeterminate`/NaN as a distinct outcome, never silently 0 and never "satisfied"
(LexiCone F24; LexiCert monitor typing). For NAVSIM's pseudo‑closed‑loop two‑stage weighting,
adopt the audit's own corrections: weight by the **simulated** endpoint not the commanded one
(F10), normalise in log space to kill the 1.92 m discontinuity (F07), and make a per‑token
failure a recorded `indeterminate` row rather than a run abort (F03).

### 3.6 Necessity / "false‑positive" handling, done structurally (Pillar 7)

EPDMS's human filter exists for a real reason — sometimes a rule must be broken (briefly enter
the oncoming lane to pass a static obstacle) — but the binary "human = 0 ⇒ agent = 1" is wrong
(F20). Handle necessity in priority order:

1. **Preferred — precise applicability.** Make the context gate exact so a *necessary*
   manoeuvre is `not_applicable`, not a violation, in the first place. (This is the shared
   predicate/monitor‑gate discipline of LexiCert F09/F52 applied at the semantic level.)
2. **Scene‑feasible baseline.** Where a violation is genuinely unavoidable, score the tier
   *relative to the scene's minimum achievable violation* `ν_i^scene` — the natural analogue
   of LexiCert's cascade value `ν_i`, asking "did the agent achieve the scene's minimum
   necessary violation?" Symmetric and graded.
3. **Fallback.** If a scene baseline is too costly, use `filter_i = max(v_agent, v_human)`
   (the audit's own F20 correction), scoring the human under *identical* inputs — never the
   asymmetric binary rule.

---

## 4. The measurement layer, hardened against the findings (Pillar 6)

The scheme is only as good as the per‑frame measurement, and the audits are a ready‑made
defect catalogue. Each requirement below cites the finding it closes.

- **One gate shared between predicate and monitor** where the semantics must coincide (e.g. a
  longitudinal gate on lateral‑clearance), explicit and documented where they must differ.
  *(LexiCert F09, F52.)*
- **One frame convention.** Place the ego box at its declared reference point; convert
  rear‑axle↔centre once and test it (IoU = 1 against the footprint builder). *(NAVSIM F02/F21;
  LexiCert F20.)*
- **One cadence.** All windows in **seconds**, converted by `dt`; read per‑frame signals per
  frame (never tile the sample‑frame traffic‑light state across the window); differentiate on
  the true time axis (no resampling seam). *(LexiCert F02, F19; NAVSIM F04.)*
- **Event‑time semantics.** Classify at‑fault collision and TTC from the track state *at the
  event*, not first‑seen; exempt overlaps already present at t=0. *(NAVSIM F06, F09.)*
- **Bounded fractions.** Assert `violation_frames ≤ applicable_frames` and test it *on the
  frozen record*, not only on a synthetic fixture. *(LexiCert F15.)*
- **Absolute references.** EP against an absolute route reference; LK as absolute lane
  geometry, not distance to a reference planner. *(NAVSIM F08, F27.)*
- **Typed stop/signal inputs.** Filter stop polygons by type; map signal enums correctly;
  carry `speed_limit_source` per scene; never apply a default limit silently.
  *(LexiCert F16, F64, F77.)*
- **Positive/negative fixtures per rule** — wait‑then‑go light, same‑lane leader, legal
  overtake into oncoming, far narrow lane — run on *every* cadence the benchmark uses.
  *(LexiCert 04 §7; LexiCone 04 §9.)*

Evidence discipline is part of the measurement, because a benchmark's credibility *is* its
reproducibility:

- Freeze every record with `SHA256SUMS`; route **every** figure/number generator through a
  `require_frozen` guard (not 13 of them bypassing it); record a real `git_commit` in every
  manifest; make a record satisfy its own index. *(LexiCert F03, F17, F18, F61.)*
- **Pre‑register** tiers, `δ_i`, `B_i`, normalisations and gates before any run; never
  redefine a gate to turn a failing run into a passing one; if you must amend, record the
  amendment and index the failing run. *(LexiCert F06, F07.)*
- Ship a **"what this score does and does not establish"** section with every result set — a
  sampled 10 Hz grid is not a continuous‑time guarantee; a `not_applicable` is not a pass;
  agreement with an external evaluator holds only on verified‑equivalent predicates. *(All
  three audits; LexiCone's Construe registry is the model.)*

---

## 5. A concrete NAVSIM instantiation (L‑EPDMS)

Map the existing EPDMS sub‑scores onto tiers, re‑graded and absolutised. This is a drop‑in
re‑aggregation of metrics NAVSIM already computes, plus three measurement repairs.

| Tier | Rules (from EPDMS) | Change vs EPDMS |
|---|---|---|
| T0 SAFETY | NC (at‑fault collision), TTC | NC becomes a **graded** severity (depth×speed×fault), not `{0,½,1}`; both measured at event time, t=0 overlaps exempt |
| T1 LEGAL | TLC (traffic light), DDC (driving direction) | TLC graded by speed‑past‑bar; DDC becomes **signed** wrong‑way progress, not unsigned off‑route displacement |
| T2 ROAD | DAC (drivable area), LK (lane keeping) | LK **absolute** lane geometry, not distance to PDM‑Closed centreline; reset across intersections |
| T3 COMFORT | HC (history comfort), EC (extended comfort) | HC history differentiated on the true axis (no seam); EC on the two rollouts as today |
| T4 PROGRESS | EP (ego progress) | EP against an **absolute** route reference, not `max`‑over‑proposals; `indeterminate` when the reference is degenerate, never silently 1 |

Then: quantise with pre‑registered `δ_i`, compare lexicographically (ground truth), and report
`S` = the budget‑checked positional scalar as the single leaderboard column. The three
measurement repairs that L‑EPDMS *requires* and that also fix NAVSIM's own audit — per‑frame
traffic‑light state, rear‑axle↔centre consistency, and traffic agents that receive light
status — are already on the NAVSIM remediation list, so the benchmark and the devkit improve
together.

A natural validation claim, in NAVSIM's own idiom: **L‑EPDMS should correlate at least as well
as EPDMS with true closed‑loop ranking, while being monotone in safety by construction** — a
planner can never climb the leaderboard by trading a collision for progress. That is a
testable, falsifiable hypothesis and the right headline experiment.

---

## 6. What this scheme establishes, and what it does not

**Establishes.** A ranking of trajectories that is (i) a function of trajectory and scene
alone, (ii) absolute and graded, (iii) strictly safety‑first across tiers, (iv) honest about
ties at a declared resolution, and (v) reproducible from hashed, pre‑registered records. The
scalar is exact (zero band), solver‑free, and float‑safe within a stated budget.

**Does not establish.** That the chosen tier order or the `δ_i` are *correct* — those are
policy inputs the rulebook makes explicit, not facts the benchmark proves (this is exactly the
honesty LexiCert owes about `τ` and `κ⁺`, and NAVSIM about `5/5/2/2/2`). Nor does it establish
anything about continuous‑time behaviour between samples, about scenes marked
`not_applicable`/`indeterminate`, or about traffic that would have reacted differently to the
real ego. And it is a *scoring* scheme: it ranks trajectories, it does not plan them — the
LexiCert/LexiCone planners are the optimisation side, and the natural bridge is to *optimise
against the same rulebook the benchmark scores with* (minimum‑violation planning), so that the
objective and the evaluation are the same object.

---

## 7. Relationship to prior work and to your three repos

**Literature.** The tier order and hierarchical comparison are Censi et al.'s *rulebooks*
(ICRA 2019). Grading within the failure set and preferring the least violation are
*minimum‑violation planning* (Tumova, Hall, Karaman, Frazzoli, Rus; Castro, Karaman,
Frazzoli; Wongpiromsarn, Karaman). The scalarisation of a lexicographic order and its exact
reproduction by a weight ladder is your own LexiCert theory (Hoffman‑bound certificate) and
the classical exact‑penalty / preemptive‑goal‑programming result behind it; the warning that a
*single* diverging weight does not suffice is LexiCone F01 and the general multi‑objective
caution in Marler & Arora's survey. The evaluation target is NAVSIM's EPDMS and nuPlan's
closed‑loop score.

**Reuse from LexiCert.** The ladder recursion, the audit‑against‑an‑exact‑reference pattern,
the freeze/`require_frozen`/`SHA256SUMS` evidence layer, and the typed `RuleCtx`/monitor
machinery — all sound, all reusable. Drop the `σ̂` instrument and the `1e-6` floor: scoring
needs neither.

**Reuse from LexiCone.** The observer's typed per‑rule results, the `NOT_APPLICABLE /
UNSUPPORTED / INDETERMINATE` outcome algebra, and above all the Construe *semantic registry*
discipline — never equate two predicates without a verified mapping — which is the right model
for L‑EPDMS claiming parity with EPDMS. Drop the single‑weight calibration claim; the benchmark
does not need calibrated weights because it does not solve.

**Reuse from NAVSIM.** The sub‑score implementations and the pseudo‑simulation rollout, with
the measurement repairs of §4. The devkit stays the harness; only the aggregation changes.

---

## 8. Minimal build order

1. **Rulebook spec + typed `RuleResult`** (§3.1–3.2): tiers, `B_i`, absolute normalisations,
   `δ_i` — pre‑registered.
2. **Lexicographic comparator + positional scalar with float‑budget assertion** (§3.3–3.4),
   and the exact‑agreement self‑audit (compute both, assert equal, freeze).
3. **Measurement repairs** (§4) on the three NAVSIM sub‑scores that are reference‑relative or
   frame/cadence‑inconsistent (EP, LK, collision/TTC event‑time), with positive/negative
   fixtures.
4. **Tier‑wise corpus aggregation** (§3.5) and the "does/does not establish" report.
5. **Headline experiment** (§5): correlation of L‑EPDMS vs EPDMS ranking with closed‑loop, and
   a monotone‑in‑safety stress test.

The first two items are small and self‑contained — they are the core contribution and carry no
data dependency — and the exact self‑audit makes the scalar trustworthy from day one.
