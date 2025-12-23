Comparison: CORBA vs PyHPP (Markdown)
=====================================

This page embeds the Markdown source file.

For *rendered* Markdown, install MyST and rebuild:

.. code-block:: bash

   python3 -m pip install --user myst-parser
   make clean html

Source:

.. only:: myst

   .. include:: comparison_corba_vs_pyhpp.md
      :parser: myst_parser.sphinx_

.. only:: not myst

   .. literalinclude:: comparison_corba_vs_pyhpp.md
      :language: text
