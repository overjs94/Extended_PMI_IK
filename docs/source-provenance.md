# Inputs, provenance and integration status

- Supplied `PMI-IK (2).zip`: extracted outside the checkout. Selected core C++/header/CMake/package files are copied unchanged to `reference/pmi_ik/`; `SHA256.json` verifies those copies. This directory is a reference snapshot, not a standalone build of the full ROS package.
- Supplied `panda_franka.urdf` and its collision STL / visual DAE files are retained under `models/`. The multi-Panda assemblies are generated from this URDF; their placements, object, rigid grasps and scenarios are new synthetic experiments.
- Supplied RAL PDF: *Real-Time Constraint-Aware Inverse Kinematics via Passivity-Based Dynamics Simulation*, Jeongseob Lee and Dongjun Lee, accepted February 2026.
- Supplied white note: *Passivity-Preserving Parallel PMI Optimization in Maximal Coordinates*, Jeongseob Lee. Its open theoretical questions are treated as open, not proven by this implementation.
- The initial large `zip files.zip` exceeded the upload tool's 32 MiB limit and was not read. The subsequently supplied PMI-IK archive was read successfully.
- Other ChatGPT conversations and the contact-contraction-transfer repository were not accessible by the available connectors. Their details are not inferred.
- Public model source locations were cross-checked against `robot-descriptions/robot_descriptions.py`; each selected upstream revision and URDF path is explicitly saved in `models/registry.json`. The fetcher downloads only from those pinned public GitHub sources using normal TLS validation.

The original uploaded `qp_solver` wrapper contains qpOASES and qpSWIFT adapters. OSQP was described by the user as another third-party dependency; its implementation was not present in that archive. The new backend uses the official `osqp` Python package. qpOASES is executed via CasADi's qpOASES plugin, not a stand-in solver. Upstream binaries retain their respective licenses (qpOASES LGPL-2.1; see installed distributions for complete notices).

External robot data is kept in ignored `models/external/`, including original URDFs, derived local-resource URDFs, source revision, file SHA256 values, downloaded licenses when present, and missing-asset diagnostics. The source registry and reproducible downloader are versioned. Missing root license files do not imply permission for unrestricted redistribution. The supplied ROS package's license field was `TODO`; no project license has been invented here.
