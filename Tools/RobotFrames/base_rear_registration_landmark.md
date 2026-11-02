# Superseded fixed-housing contact

The fixed gray-base rear point formerly described here was the wrong physical
target. The requested contact is the **lowest blue cover**, at the pan joint,
on the moving `ur_shoulder_link`. It is now the **first** contact, followed by
Shoulder, Elbow, Wrist 2 and Wrist 3.

See [the blue base-cover derivation](base_blue_cover_registration_landmark.md)
for the current coordinates and CAD surface proof. Untouched legacy four-cover
defaults and the superseded five-point gray-base defaults are migrated to this
ordered five-blue-cover set. Custom coordinates are preserved for explicit
configuration.
