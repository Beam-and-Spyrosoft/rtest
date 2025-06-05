# Copyright 2025 Spyrosoft S.A.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# @file      conf.py
# @author    Sławomir Cielepak (slawomir.cielepak@gmail.com)
# @date      2025-05-14
#
# @brief Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
#
# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import time

project = 'rtest'
author = 'Spyrosoft Synergy S.A.'
copyright = '{}, {}'.format(time.strftime('%Y'), author)

# The master toctree document.
master_doc = 'index'
source_suffix = '.rst'

language = 'en'

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
extensions = [
    'sphinx.ext.graphviz',
    'sphinx.ext.intersphinx',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinx.ext.todo',
    'sphinx_rtd_theme',
    'myst_parser',
    'breathe',
    'exhale'
]

source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'markdown',
    '.md': 'markdown',
}

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

root_doc = 'index'

# allow to build Unicode chars
latex_engine = 'xelatex'

autosectionlabel_prefix_document = True
hoverxref_auto_ref = True
hoverxref_roles = [
    'term',
]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_extra_path = ['static']
html_logo = "logo_v.png"
html_theme_options = {
    'logo_only': True,
    'display_version': True,
}

# -- MyST-Parser specific options (optional) ---------------------------------
myst_enable_extensions = [
    "amsmath",          # For math equations
    "colon_fence",      # For .. directive like blocks
    "deflist",
    "dollarmath",       # For inline and block math
    "html_admonition",
    "html_image",
    "linkify",          # Autolink URLs
    "replacements",
    "smartquotes",
    "substitution",
    "tasklist",
]
myst_heading_anchors = 3 # Auto-generate header anchors up to level 3


# -- Breathe Configuration ---------------------------------------------------
# Tells Breathe where to find the Doxygen XML output.
# The path is relative to the conf.py file.
breathe_projects = {
    "rtest": "xml/" # Path to Doxygen XML output folder
}
breathe_default_project = "rtest"


# -- Exhale Configuration ----------------------------------------------------
# Setup Exhale to generate API documentation from Doxygen output.
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",    # Folder in `doc/` to generate API files
    "rootFileName":          "library_root.rst", # Root file for the API section
    "doxygenStripFromPath":  "..",      # Strips '..' from Doxygen paths
    # Suggested arguments
    "createTreeView":        True,      # Create a tree view for better navigation
    # TIP: exhaleExecutesDoxygen is True by default, you can skip this line
    "exhaleExecutesDoxygen": True,
    # "exhaleDoxygenStdin":    "INPUT = ../include ../src" # Basic Doxygen config
    # It's usually better to use a Doxyfile for more complex projects:
    "exhaleUseDoxyfile":     True,
    # "exhaleDoxygenDoxyfile": "Doxyfile", # Path to your Doxyfile relative to conf.py
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'

todo_include_todos = True