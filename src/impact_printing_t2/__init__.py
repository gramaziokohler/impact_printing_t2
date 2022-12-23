"""
********************************************************************************
impact_printing_t2
********************************************************************************

.. currentmodule:: impact_printing_t2


.. toctree::
    :maxdepth: 1


"""

from __future__ import print_function

import os


__author__ = ["Kunal Chadha"]
__copyright__ = "Gramazio Kohler Research"
__license__ = "MIT License"
__email__ = "chadha@arch.ethz.ch"
__version__ = "0.1.0"


HERE = os.path.dirname(__file__)

HOME = os.path.abspath(os.path.join(HERE, "../../"))
DATA = os.path.abspath(os.path.join(HOME, "data"))
DOCS = os.path.abspath(os.path.join(HOME, "docs"))
TEMP = os.path.abspath(os.path.join(HOME, "temp"))


__all__ = ["HOME", "DATA", "DOCS", "TEMP"]
