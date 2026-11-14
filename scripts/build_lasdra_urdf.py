#!/usr/bin/env python3
"""Regenerate the reviewed URDF transcription of the supplied LASDRA model."""
from pathlib import Path
from extended_pmi_ik.lasdra import lasdra_xml

root=Path(__file__).resolve().parents[1]
(root/'models/lasdra.urdf').write_text(lasdra_xml()+'\n')
print('Wrote models/lasdra.urdf; validate against the original C++ using tests/test_trajectories.py')
