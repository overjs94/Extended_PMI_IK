# Solver dependencies

The new core installs pinned OSQP and CasADi distributions through `requirements.lock`.
CasADi provides the actual qpOASES conic plugin; no checked-in binary or fabricated qpOASES wrapper is used.
The original uploaded ROS source expects additional sibling third-party trees; those are not reconstructed here.
Public robot assets are downloaded separately by `scripts/fetch_models.py` and retain their own provenance and licenses.
