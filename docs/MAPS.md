# Maps and examples

The repository includes the four occupancy-grid map pairs already present in the original team repository:

- `intent.pgm` / `intent.yaml`
- `mapa_aula2.pgm` / `mapa_aula2.yaml`
- `mapa_circuit_classe.pgm` / `mapa_circuit_classe.yaml`
- `mapa_passadis.pgm` / `mapa_passadis.yaml`

The `.yaml` file provides the resolution, map origin and thresholds used by ROS `map_server`. Its `image:` field is a relative PGM filename.

## Other files supplied with the coursework

A separate `mapes` ZIP also contained a `mapa_boticelli_real` map, a landmarks text file and duplicate files (some with `(1)` suffixes). These extra binary map assets are **not yet in this GitHub branch**. Their inclusion is optional for understanding the six-node architecture, and they should be checked before publication for site information.

If the extra map is later added, upload each PGM, its matching YAML and only the landmarks files intended for public release. Avoid uploading duplicated `(1)` variants.
