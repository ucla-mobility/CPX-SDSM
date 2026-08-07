# Bag placement

The project does not duplicate the large source bags. Copy only the two small
structured tracking recordings when replay validation is needed:

- `vehicle/2026-07-22-17-36-56.bag`
- `infrastructure/2026-07-22-16-14-46.bag`

Live mode does not use this directory.

For the optional raw detector profile (`ETX_SOURCE_VARIANT=raw`), use the full
recordings instead. Both original tracking workspaces already provide
`autoware_msgs/DetectedObjectArray`; the project intentionally does not rebuild
that large dependency tree.
