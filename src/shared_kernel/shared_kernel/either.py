"""Tipo Either genérico, para expresar "esto puede fallar" sin excepciones."""

from dataclasses import dataclass
from typing import Generic, TypeVar, Union

L = TypeVar("L")
A = TypeVar("A")


@dataclass(frozen=True)
class Left(Generic[L, A]):
    value: L

    def is_left(self) -> bool:
        return True

    def is_right(self) -> bool:
        return False


@dataclass(frozen=True)
class Right(Generic[L, A]):
    value: A

    def is_left(self) -> bool:
        return False

    def is_right(self) -> bool:
        return True


Either = Union[Left[L, A], Right[L, A]]


def left(value: L) -> Either[L, A]:
    return Left(value)


def right(value: A) -> Either[L, A]:
    return Right(value)
