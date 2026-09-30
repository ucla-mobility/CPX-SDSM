# Ideas — future local trustworthiness factors

- **Model confidence:** Trained estimation of the model on the certainty that
  and object is actually existing. This is a direct correlation to trustworthiness.
  If the model does not trust its own estimation it is likely wrong.

- **Absolute object size** (distinct from the existing *shape*-plausibility
  factor, which checks dimensions against the class prior): small objects
  such as pedestrians or traffic cones are intrinsically harder to detect
  than large ones like trucks — fewer sensor returns, easier to occlude,
  smaller image footprint. A size factor would discount small classes (or
  small measured volumes) independently of whether their shape fits the
  class.

- **Temporal shape consistency:** compare an object's measured dimensions
  across consecutive frames. Large, unexpected size changes for the same
  track may indicate that the object is no longer being recognized reliably.

- **Spatiotemporal consistency:** compare an object's position and motion
  across consecutive frames. Implausible jumps from one position to another
  may indicate an unstable detection or incorrect track association.

- **Sensor availability and health:** reduce trust when a contributing sensor
  becomes unavailable or reports degraded operation, since detections may
  then rely on incomplete or lower-quality observations.

- **High ISO / low-light conditions:** high ISO, low illumination, or similar
  exposure conditions make the camera less reliable.

- **High reflectivity / lidar intensity:** detect unusually reflective returns
  that may be caused by glass, wet surfaces, or similar materials and may
  distort lidar-based perception.

- **Bad Weather:** e.g. fog / rain which blocks the view of the sensors.
  This leads to sensor degradation.

- **Ground and map relationship:** check whether an object occupies a
  physically and semantically plausible location. Floating objects, objects
  inside walls, or objects in inaccessible locations may be unreliable
  detections.

- **Overlapping bounding boxes:** check wether a bounding box overlaps with
  another one. Since this is physically impossible, the positional accuracy
  is seemingly low. 

These factors are all signals that an object is, or may be becoming, less
reliably recognized.
