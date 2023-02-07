# This file is copied from scipy _rotation.pyi with minimal modifications
#
#
# Copyright (c) 2001-2002 Enthought, Inc. 2003-2023, SciPy Developers.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import annotations
from typing import TYPE_CHECKING, Union, Tuple, Optional, Sequence, Any
import numpy as np

if TYPE_CHECKING:
    import numpy.typing as npt

_IntegerType = Union[int, np.integer]

def __getattr__(name: str) -> Any: ...

class Rotation:
    def __init__(
        self, quat: npt.ArrayLike, normalize: bool = ..., copy: bool = ...
    ) -> None: ...
    @property
    def single(self) -> bool: ...
    def __len__(self) -> int: ...
    @classmethod
    def from_quat(cls, quat: npt.ArrayLike) -> Rotation: ...
    @classmethod
    def from_matrix(cls, matrix: npt.ArrayLike) -> Rotation: ...
    @classmethod
    def from_rotvec(cls, rotvec: npt.ArrayLike) -> Rotation: ...
    @classmethod
    def from_euler(
        cls, seq: str, angles: Union[float, npt.ArrayLike], degrees: bool = ...
    ) -> Rotation: ...
    @classmethod
    def from_mrp(cls, mrp: npt.ArrayLike) -> Rotation: ...
    def as_quat(self) -> np.ndarray: ...
    def as_matrix(self) -> np.ndarray: ...
    def as_rotvec(self) -> np.ndarray: ...
    def as_euler(self, seq: str, degrees: bool = ...) -> np.ndarray: ...
    def as_mrp(self) -> np.ndarray: ...
    @classmethod
    def concatenate(cls, rotations: Sequence[Rotation]) -> Rotation: ...
    def apply(self, vectors: npt.ArrayLike, inverse: bool = ...) -> np.ndarray: ...
    def __mul__(self, other: Rotation) -> Rotation: ...
    def inv(self) -> Rotation: ...
    def magnitude(self) -> Union[np.ndarray, float]: ...
    def mean(self, weights: Optional[npt.ArrayLike] = ...) -> Rotation: ...
    def reduce(
        self,
        left: Optional[Rotation] = ...,
        right: Optional[Rotation] = ...,
        return_indices: bool = ...,
    ) -> Union[Rotation, Tuple[Rotation, np.ndarray, np.ndarray]]: ...
    @classmethod
    def create_group(cls, group: str, axis: str = ...) -> Rotation: ...
    def __getitem__(self, indexer: Union[int, slice, npt.ArrayLike]) -> Rotation: ...
    @classmethod
    def identity(cls, num: Optional[int] = ...) -> Rotation: ...
    @classmethod
    def random(
        cls,
        num: Optional[int] = ...,
        random_state: Optional[
            Union[_IntegerType, np.random.Generator, np.random.RandomState]
        ] = ...,
    ) -> Rotation: ...
    @classmethod
    def align_vectors(
        cls,
        a: npt.ArrayLike,
        b: npt.ArrayLike,
        weights: Optional[npt.ArrayLike] = ...,
        return_sensitivity: bool = ...,
    ) -> Union[Tuple[Rotation, float], Tuple[Rotation, float, np.ndarray]]: ...

class Slerp:
    def __init__(self, times: npt.ArrayLike, rotations: Rotation) -> None: ...
    def __call__(self, times: npt.ArrayLike) -> Rotation: ...
