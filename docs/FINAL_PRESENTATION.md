# Bot-ticelli — final course presentation

This document summarizes the team's **15-slide final presentation** (*PFINAL RA MÒBIL.pdf*, Advanced Robotics, UPC). The original PDF is available in the course hand-in and should be added here separately only if the team agrees to redistribute it.

## Goal and architecture (slides 1–5)

Bot-ticelli is a ROS-based mobile drawing system developed for the TurtleBot3 Waffle. Its purpose is to map or load an environment, localize the robot, detect a drawing with computer vision, generate a trajectory and follow the trajectory on a real floor surface.

The architecture combines external ROS modules (`slam_gmapping`, `map_server`, `amcl`) and six team-written nodes. The pipeline uses `/scan`, `/odom`, `/tf`, `/map`, image inputs, contour topics, `/drawing_path` and `/cmd_vel`.

## Localization and free-space selection (slides 6–7)

For unknown environments, GMapping uses the LiDAR and odometry to build an occupancy map. For saved maps, AMCL estimates the robot pose using a particle filter. A free-space explorer uses occupancy information, integral-image checks, connected regions and multi-source BFS-style clearance calculations to select an area where the drawing fits.

## Vision and path generation (slides 8–9)

The vision process includes grayscale conversion, Gaussian filtering, Otsu thresholding, morphological operations, contour extraction and contour simplification (Ramer–Douglas–Peucker).

The detected outline is normalized, scaled to physical dimensions and transformed into a path in the map reference frame. Path points are redistributed to make movement more consistent.

## Navigation and drawing (slide 10)

The robot approaches the start of the drawing and aligns with the initial path tangent. During drawing, it computes distance and angular errors to the next waypoint, adapts its linear/angular velocities and reduces speed for sharp turns. The presented system includes a pause/stop response to detected frontal obstacles.

## Demonstrations (slides 11–13)

The following two distinct YouTube URLs are embedded in the team's presentation:

- [Figure demonstration — Pikachu](https://youtu.be/Dro_8_x7c0A), on slides 11–12.
- [Figure demonstration — star](https://youtu.be/gTl-xQZ3lTM), on slide 13.

**Note:** Slide 12 displays a thumbnail captioned “figura fluida” but embeds the *same video URL* as slide 11. This document intentionally does not invent a third link. These URLs were extracted from the PDF annotations; playback has not been independently verified.

## Results and limitations (slide 14)

The team reported a functional integrated ROS system and physical drawing demonstrations on the floor. The presentation also identifies several limitations:

- Vision depends on color contrast and lighting.
- The drawing paper/workspace requires manual preparation.
- The improvised marker mount changes pressure and angle, affecting line quality.
- Odometry and surface friction can cause trajectory deviations.
- Excessive simplification reduces drawing fidelity; insufficient simplification makes turns difficult.
- Complex corners require slower movement to avoid rounding.

The final slides propose possible future applications in industrial floor marking, events, construction/interior layout and art. These are **potential uses**, not deployments achieved by the project.

## Team

Blanca Mira, Laida Queral, Olga Obiols, Pablo Abella and Raul Castillo.
