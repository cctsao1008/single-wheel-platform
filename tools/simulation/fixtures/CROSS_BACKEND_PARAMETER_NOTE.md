# Cross-backend parameter consistency

For issue #16, a simulator-neutral experiment is meaningful only when every backend receives the same physical parameter intent. The original reduced-model correlation fixture deliberately used convenient synthetic inertias and was not geometrically realizable by the bootstrap Webots solids. The cross-backend suite therefore uses `synthetic-rigidbody-correlation.json`, whose reduced-model inertias are derived from the same synthetic box/cylinder geometry used by Webots.

This is a simulation consistency rule, not physical ONE V2 evidence.
