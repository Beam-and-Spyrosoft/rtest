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
release = '0.1.0'

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
    'sphinx_rtd_theme',
    'sphinx_autorun',
    'sphinx_tabs.tabs',
    'notfound.extension',
    'myst_parser',
    'breathe'
]

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
html_logo = "logo.png"
html_theme_options = {
    'logo_only': True,
    'display_version': True,
}
