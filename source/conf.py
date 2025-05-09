# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Hands-On ROS 2 for Robotics Programming'
copyright = '2025, fishros.org'
author = 'fishros'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'recommonmark',  # Replace 'recommonmark' with this
    # 'sphinx_markdown_tables',
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.todo',
    'sphinx.ext.autosummary',
]

# Add these settings for better table support
# source_suffix = {
#     '.rst': 'restructuredtext',
#     '.md': 'markdown',
# }

templates_path = ['_templates']
exclude_patterns = []
release = 'v1'


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
