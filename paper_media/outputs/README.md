# paper_media/outputs

Figures, key frames and videos for the paper and the website, one directory
per media item. Like `paper_results/`, every item carries its provenance; unlike
`paper_results/`, nothing here feeds a paper number.

Rules:

- Media come from dedicated, unscored sessions only. `package` refuses a
  session that appears in any `paper_results/batches/*/batch.yaml`.
- Each item is produced by one story file in `paper_media/stories/`, named
  after its `media_id`. Re-running `package` replaces the item's directory.
- Figures carry no camera images, captions or prompt excerpts. Those are added
  by hand in the layout tool. The numbered ticks on a figure's time axis are the
  key frame numbers in `frames/`: tick 3 is every `frames/<camera>/03_t...` file.
- Labels are grouped: one shaded band per executor step, one label per burst of
  monitor armings ("9 monitors armed"), fires named individually, and on the
  WatchDog lane only look(), observation and the verdict. The bus has no text;
  marker shapes (legend under the axis) say what each event is.

## Layout

```text
paper_media/outputs/
  README.md
  <media_id>/                 <UTC date of the session>_<scenario>_<what it shows>
    media.yaml                provenance, labels, tags, window, thresholds, file list
    story.yaml                the story exactly as used
    figure/
      <media_id>_double.svg   7.16 in, two column (SVG only, for editing in PowerPoint)
      <media_id>_single.svg   3.5 in, one column
    frames/
      frames.csv              key, camera, file, requested and actual session time, gap, wall_time_iso, source
      <camera>/<n>_t<session seconds>s.png|jpg
    video/
      <media_id>.mp4          1920 x 1080, 30 fps, speed badge always on screen
      <media_id>_poster.jpg
```

Frame cameras:

| camera folder | source | resolution |
|---|---|---|
| `recorder` | frame recorder on `/recorder_rgb`, a recording-only camera | 1280 x 720 |
| `main` | frame recorder on `/camera/depth/rgb`, the agents' own view (optional second instance) | 640 x 360 |
| `replay_cell_<label>` | Isaac replay from the bag, same pose and lens as the cell camera | 1920 x 1080 |
| `replay_main_<label>` | Isaac replay from the bag, oblique view | 1920 x 1080 |

Isaac replay frames rebuild the robot from `/joint_states`, the parts from
`/tf`, and injected objects from `ground_truth_injections.jsonl` (visitors move
linearly between their logged phase points). They match the recorder frames in
pose and timing; see `paper_media/isaac_replay/README.md`.

## Items

| media_id | session | kind | window [s] | content |
|---|---|---|---|---|
| 20261102_S3_c1_observation | 20261102T195738Z | observation_monitoring | 330-690 | foreign cube and visitor box, both recorded; ground truth overlay |
| 20261102_S3_natural_place_inverted_u_yellow | 20261102T195738Z | conformance_monitoring | 740-952 | natural collision on place_inverted_u_yellow, five fires, non_conformant |
| 20261102_S3_c1_conformant_run | 20261102T204534Z | observation_monitoring | 210-856 | conformant run; cube recorded 430 s late, visitor box missed; ground truth overlay |
| 20261102_S3_t1_conformance_u_brown | 20261102T195738Z | conformance_monitoring | 220-352 | pick and place of u_brown; one border watch fire, checked by look() |
| 20261102_S3_t2_conformance_u_brown | 20261102T204534Z | conformance_monitoring | 214-361 | pick and place of u_brown; no fire |
| 20261102_S1_c1_conformance_u_brown | 20261102T222941Z | conformance_monitoring | 52-202 | S1 library monitors; pick and place of u_brown; no fire |
| 20261102_S1_natural_place_line_green | 20261102T222941Z | conformance_monitoring | 388-770 | line_green slip, failed retry knocks the assembly, non_conformant; cube and visitor box missed |
| 20261102_S2_c1_conformant_run | 20261102T230701Z | observation_monitoring | 48-555 | conformant S2 run; cube and visitor box missed |

Templates: `paper_media/stories/template_conformance.yaml` and `template_observation.yaml`.

## Commands

```bash
P=~/.venvs/paper_media/bin/python
$P -m paper_media.replay.cli package paper_media/stories/<media_id>.yaml            # figure, frames, video
$P -m paper_media.replay.cli package paper_media/stories/<media_id>.yaml --no-video # figure and frames only
```
