# -- Project information -----------------------------------------------------
project = 'rosgraph_monitor'
author = 'rosgraph_monitor contributors'
release = '0.0.0'


# -- General configuration ---------------------------------------------------
extensions = [
	'sphinx.ext.autodoc',
	'sphinx.ext.intersphinx',
	'sphinx.ext.todo',
	'sphinx.ext.viewcode',
	'breathe',
    'myst_parser'
]

templates_path = ['_templates']
source_suffix = ['.rst']
master_doc = 'index'
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# Show TODOs if the todo extension is enabled
todo_include_todos = True
