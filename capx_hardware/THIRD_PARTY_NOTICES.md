# Bundled source provenance

`fr3_capx` includes code adapted from the MIT-licensed CaP-X project at commit
`53e9966d7a8e2fa7494676772bccc35280f5c0ed`. Its license is preserved in
`THIRD_PARTY_LICENSES/CaP-X-MIT.txt` and module source headers.

The existing FR3 fieldkit and camera integration source is copied byte-for-byte
from the current workspace's separately preserved field packages. Exact original
relative source paths and SHA-256 values appear in `evidence/vendor-provenance.json`.
This addition does not relicense the original repository or its device SDKs.
libfranka and librealsense are external build/runtime dependencies; their binaries
are not redistributed here.

The `capx_autoring` policy and native assets are ported from the same existing
workspace/repository `field_demo_random_20261022/src`. Source/asset hashes and
namespace/phase instrumentation changes are listed in
`evidence/autoring-upstream-port.json`. Original asset licenses/provenance are
retained beside the assets. No libfranka/librealsense binaries or external model
weights are bundled. Geometry NPZs are class CAD assets, not runtime truth poses.
