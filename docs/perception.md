# Perception: what the fleet can recognise, and how we know

The detector is open-vocabulary: YOLOE is handed text at startup and finds whatever that
text describes. That makes adding an object cheap — no dataset, no training — and moves
the entire engineering problem into two questions this document answers. *What words?*
and *how do we know they work?*

Everything lives in one table, [`adapters/perception/catalog.py`](../adapters/perception/catalog.py).
The inference sidecar binds it, the adapters filter against it, the backend describes it
to the dashboard, and the settings dialog renders it. There is no second list anywhere.

## The catalog

| Class | Label | Prompts | Floor |
|---|---|---|---|
| `rubber_duck` | Rubber duck | `yellow rubber duck toy`, `rubber duck` | 0.25 |
| `wooden_block` | Wooden block | `wooden block`, `wooden toy block` | 0.35 |
| `disc_cone` | Disc cone | `orange plastic saucer`, `orange plastic disc`, `saucer cone` | 0.20 |
| `filament_spool` | Filament spool | `roll of black filament`, `spool of black plastic wire`, `filament spool` | 0.25 |
| `pool_noodle` | Pool noodle | `pool noodle`, `swimming pool noodle`, `blue foam tube` | 0.25 |

A class owns several prompts because they are alternative descriptions of one object, not
distinct classes; the sidecar binds them all and maps whichever matched back to the class
name before the detection leaves the process.

## Why the wording is calibration data, not documentation

YOLOE's text encoder is literal in ways that are genuinely hard to predict, so prompts
were measured one at a time against the reference photographs in `images/` rather than
chosen for readability. The clearest case is the disc cone. Scores on `disc_cone.jpg`,
YOLOE-26n-seg at 640 px:

| Prompt | Score |
|---|---|
| `orange plastic saucer` | **0.69** |
| `orange plastic disc` | 0.44 |
| `saucer cone` | 0.18 |
| `orange plastic cone` | 0.09 |
| `cone`, `traffic cone`, `orange cone`, `sports cone`, `disc cone` | nothing at all |

Every prompt containing the bare word "cone" fails, because the embedding is dominated by
tall highway cones and the object is a flat saucer. The obvious name for the object is the
one that does not work.

The filament spool showed the same effect more mildly: `roll of black filament` reaches
0.38 where `spool` alone reaches 0.06 and `cable reel` finds nothing.

## Why score floors are per class

The five objects are not equally easy, and the spread is nearly 3x:

| Class | Best score on its reference photo |
|---|---|
| `wooden_block` | 0.97 |
| `pool_noodle` | 0.89 |
| `disc_cone` | 0.69 |
| `filament_spool` | 0.38 |

A single global threshold set for blocks would never see a spool; one set for spools
would flood the map with anything vaguely block-shaped. So each class carries the floor
its own evidence supports.

`filament_spool` is the one real trade-off in the table. Its floor sits as low as it can
without labelling every dark desk object a spool, and it is the class most likely to need
revisiting from field frames.

## Capture floor vs. display floor

One number used to answer two unrelated questions, which is why an operator who raised a
threshold went on seeing low-confidence detections. They are now separate:

| | Capture floor | Display floor |
|---|---|---|
| Question | What may the model report at all? | What does the operator count as real? |
| Lives in | `catalog.py`, sent to the sidecar per request | `settings.json`, enforced by the backend |
| Changes when | prompts are re-measured | an operator drags a slider |
| Scope | fleet-wide (one model, one answer) | fleet default, overridable per robot |

The sidecar enforces only the capture floor. The backend enforces the display floor
against its own stored detections, in `reapply_detection_floors()`, on every settings
save. Three things follow, and all three were broken before the split:

- **Immediate.** A saved floor is a pass over a dict on the backend. No robot is
  involved, so there is no settings-poll delay and no way for one robot to be enforcing
  last week's threshold while the rest of the fleet has moved on.
- **Retroactive.** Raising a floor hides markers already on the map, rather than only
  affecting objects detected from that moment on.
- **Reversible.** Entities are hidden, not deleted, and robots deliberately keep
  capturing below the display floor — so lowering a floor again is answered from cache.

`capture_floors()` derives what the robots are actually asked for: the *lowest* floor
anyone wants, per class. Raising a display floor therefore never changes it. Lowering one
*below* the catalog floor does, and that is the single case that still has to reach the
robots and still cannot be retroactive — those frames were never inferred on.

Visibility is judged on `best_score`, the strongest score an entity ever produced, not
its newest one. A model's confidence in a stationary object wanders by a few points frame
to frame, and filtering on the live score makes a marker sitting near its floor blink.

`detection_sensitivity` is vestigial. The sidecar ignores the `X-SwarmDeck-Confidence`
header for any class that arrives with an explicit floor, and capture floors now cover
every catalog class, so the setting is carried for compatibility and decides nothing.

## Segmentation, and why it is not cosmetic

The model is a `-seg` checkpoint, so each detection carries an outline as well as a box.
That outline is what makes a depth reading trustworthy. Measured mask area as a fraction
of the same detection's bounding box:

| Object | Mask / bbox |
|---|---|
| Wooden block (face-on) | 73–99% |
| Disc cone | 74% |
| Filament spool | 49–85% |
| **Pool noodle (diagonal)** | **27%** |

A diagonal pool noodle occupies barely a quarter of its own bounding box. Sampling depth
across that box means three quarters of the sample is the floor behind the object, and
the median lands somewhere between the two — a marker placed at neither. `depth_projection.py`
therefore rasterizes the outline and reads only the object's own pixels, falling back to
a central inset of the box when no usable mask arrived. `adapters/test/test_depth_projection.py`
pins this with a synthetic diagonal: box-only sampling reports the 6 m wall, outline
sampling reports the 2 m object.

## Verifying a change

The unit tests mock the model away, deliberately — adapters must be testable without
PyTorch. Prompts and floors are claims about a neural network, so they have their own
suite that runs the real weights:

```sh
docker compose build duck_detector
docker run --rm \
  -v swarmdeck_duck_detector_models:/models \
  -v "$PWD:/app" -e YOLO_CONFIG_DIR=/tmp/ul \
  --entrypoint bash swarmdeck-duck-detector:cpu \
  -c "pip install -q pytest && python3 -m pytest /app/tests/perception -q"
```

It asserts that each reference photograph yields its own class with a usable outline,
that the per-request class filter excludes everything else, and that sensitivity stays
monotonic. Run it after editing any prompt or floor.

## Adding a class

1. Photograph the object and commit the frame to `images/`.
2. Sweep candidate prompts **one at a time** — prompts compete when bound together, so a
   joint run cannot tell you which wording is carrying the result. Do not stop at the
   first wording that works; the cone case above cost 0.6 confidence.
3. Add the `TargetClass` to the catalog with the floor its measurements justify.
4. Add the image to `EXPECTED` in `tests/perception/test_catalog_recall.py` and run it.
5. Pick a marker colour in `ui/src/lib/stores/detection.svelte.ts`.

Nothing else needs touching: the settings dialog, the protocol, the camera overlay and
the map markers all read the catalog through the backend.

## Operator review: what earns a place on the shared map

A score floor answers "is this a real detection?". It cannot answer "is this the same
duck I already have, or a second one?" — that is a question about geometry and identity,
and the detector has no way to know. The review pipeline (`detect/review.py`) is where an
operator answers it.

Two stores, deliberately separate:

| | `_detections` | `review_store` |
|---|---|---|
| Holds | live camera tracks | validated map objects |
| Keyed by | `{robot_id}:{track_id}` | its own entity id |
| Lifetime | retracted when the object leaves frame | outlives every sighting |
| Position | the latest frame | mean of all accepted observations |

Each located sighting is triaged against the confirmed entities of its own class:

- **within `detection_same_radius_m`** (0.5 m) — the object already on the map. Folded in
  silently, its centroid re-averaged. This is the overwhelming majority: a robot parked in
  front of a duck emits a sighting every frame, and prompting per frame would make the
  queue worthless. **An object on the map never asks again.**
- **out to `detection_ask_radius_m`** (1.5 m) — genuinely ambiguous. Raised as a proposal
  with the merge *pre-suggested* and the distance named, so the operator answers "same or
  different" instead of re-deriving it from coordinates.
- **beyond** — proposed as a new object.

The operator answers accept / ignore / merge. **Ignore writes a suppression zone**, which
is what separates it from dismiss: without one the next frame re-proposes the same object
and the operator answers forever. Repeat sightings near a pending proposal strengthen that
one item rather than queueing near-duplicates, and the pending list is capped by dropping
the *weakest* evidence, so a mislabelling detector cannot bury the one real finding.

Position is a running mean over every accepted observation, kept as sums so it is O(1) and
covers all evidence rather than a recent window. One monocular range estimate is worth
little; twenty from two robots at different angles put the marker where the object is.

Nothing is hidden from the map while it waits. Confirmed objects draw as a solid disc with
a ring, pending ones as a dashed hollow ring — so an unreviewed guess is visible but never
mistakable for something a person accepted.

## Known limits

- The reference photographs are hand-held close-ups. A pass proves the prompt binds to
  the right concept; it is not a field accuracy figure for a robot camera across a room.
- Distractor confusion is real and unfixed. A plush unicorn scores 0.53 as `rubber_duck`,
  and a small desk object scores 0.29 as `filament_spool`. Both sit above their floors.
  Operator review is the backstop for exactly this: a confident wrong label reaches the
  queue, not the map.
- The review radii are metric and class-independent. Two objects of one class genuinely
  closer than `detection_same_radius_m` will merge into a single entity with no way to
  split them; `detection_forget` is the only recourse.
- Everything above was measured on YOLOE-26n-seg (nano) on CPU. The s/m/l checkpoints
  were compared on these images with a first-pass prompt set and were not better — 26l
  found neither the cone nor the spool where 26n found both — so nano stayed. That
  comparison predates the calibrated prompts above and has not been repeated with them;
  a larger model may yet win, and `SWARMDECK_YOLOE_MODEL` switches it.
