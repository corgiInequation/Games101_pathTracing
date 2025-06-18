# Games101 Experiment 7

## Experiment Content

- Since Whitted-style ray tracing only handles specular materials and approximates rough surfaces using the Blinn-Phong model, it lacks accuracy for complex surfaces. In this experiment, we use the rendering equation and implement path tracing to achieve more realistic results.
- The main task is to implement the `castRay` function:
  - First, compute the intersection between the ray and scene objects. If there is no intersection, return black.
  - If an intersection is found, handle it in two cases:
    - If the ray directly hits a light source, return the emitted value of the light.
    - If the ray hits a non-emissive object, compute two components:
      - The direct lighting from the light source to the object.
      - The indirect lighting resulting from reflected or refracted rays reaching other surfaces.
    - Monte Carlo integration is used to approximate the rendering equation.
  - Finally, return the computed color.

- Notes:
  - In the Cornell Box model used in this experiment, the right wall is infinitely thin. This can lead to cases where light passes through the wall and causes `texit <= tenter` due to floating-point precision errors. To avoid this, a small epsilon value should be added to `texit`.

## Experiment Result

- <img src="https://github.com/corgiInequation/Games101_pathTracing/blob/main/image.png" alt="path-tracing-result" width="67%" />
- sssssssssssssss

