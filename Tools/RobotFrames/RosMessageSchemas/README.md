# Fixture preview wire schemas

These definitions were read from the exact ROS Jazzy image used by the fixture
backend validation, without starting a ROS node:

`sha256:752aa09527488fce5baaa35ebbff1aad52c6e82e72a302db4f14ded79f2f615b`

`manifest.json` records every dependency and its SHA-256. The original 23
MoveIt/MTC definitions also match the installed Nordbo backend byte for byte.
Compatible standard messages use the ROS TCP Connector's existing classes.

Jazzy adds a polygon to `shape_msgs/SolidPrimitive`, and texture/UV/mesh-file
fields to `visualization_msgs/Marker`. The older connector classes omit these
wire fields. Exact nested serializers therefore use `FixtureShape` and
`FixtureVisualization` namespaces. They intentionally have no global registry
hook, preserving existing connector registrations.

Regenerate the new serializers using PowerShell 7:

```powershell
./Tools/RobotFrames/generate_fixture_preview_messages.ps1
```

The script invokes the cached connector's official `MessageAutoGen`, then
applies only private namespace and registry-hook changes. It verifies schema
hashes before generation.

`generate_fixture_solution_cdr.py` creates the independent ROS Python CDR
fixture used by `FixtureSolutionMessageTests`. It exercises the newer nested
fields and creates no ROS connection. The committed fixture records the
schema-derived alignment padding and the 16 unused trailing allocation bytes
in that ROS serialization; the schema-complete prefix also deserializes in ROS.
