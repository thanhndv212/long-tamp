HPP-Manipulation: Complete Motion Plan (Markdown)
=================================================

This page embeds the Markdown source file.

For *rendered* Markdown, install MyST and rebuild:

.. code-block:: bash

   python3 -m pip install --user myst-parser
   make clean html

Source:

.. only:: myst

   .. include:: hpp_manipulation_complete_motion_plan.md
      :parser: myst_parser.sphinx_

.. only:: not myst

   .. literalinclude:: hpp_manipulation_complete_motion_plan.md
      :language: text
