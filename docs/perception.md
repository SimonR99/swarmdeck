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
its own evidence supports. The dashboard's sensitivity slider scales all of them
together, and 0.55 — the default — is the point where each class sits exactly on its
calibrated floor, which is the behaviour the control had when there was only one class.

`filament_spool` is the one real trade-off in the table. Its floor sits as low as it can
without labelling every dark desk object a spool, and it is the class most likely to need
revisiting from field frames.

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

## Known limits

- The reference photographs are hand-held close-ups. A pass proves the prompt binds to
  the right concept; it is not a field accuracy figure for a robot camera across a room.
- Distractor confusion is real and unfixed. A plush unicorn scores 0.53 as `rubber_duck`,
  and a small desk object scores 0.29 as `filament_spool`. Both sit above their floors.
- Everything above was measured on YOLOE-26n-seg (nano) on CPU. The s/m/l checkpoints
  were compared on these images with a first-pass prompt set and were not better — 26l
  found neither the cone nor the spool where 26n found both — so nano stayed. That
  comparison predates the calibrated prompts above and has not been repeated with them;
  a larger model may yet win, and `SWARMDECK_YOLOE_MODEL` switches it.
