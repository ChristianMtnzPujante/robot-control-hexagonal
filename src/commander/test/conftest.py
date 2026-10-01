import pytest

from cell_fixtures import make_tree


@pytest.fixture
def tree(tmp_path):
    """Compila células sobre un árbol descriptions/ de juguete (ver
    `cell_fixtures.py`)."""
    return make_tree(tmp_path)
