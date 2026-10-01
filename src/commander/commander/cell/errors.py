"""Error común de toda la descripción de célula (formatos, referencias,
reglas entre piezas). Es un ValueError: un fichero mal escrito es un dato
inválido."""


class InvalidCellError(ValueError):
    """La descripción de la célula, o una de sus piezas, está incompleta o
    es incoherente. El mensaje dice dónde."""
