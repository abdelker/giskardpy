from __future__ import annotations

import functools
from typing import overload, Union, Iterable, Tuple, Optional, Callable, List, Any, Sequence
import numpy as np
import casadi as ca  # type: ignore
import geometry_msgs.msg as geometry_msgs

from giskardpy.my_types import PrefixName

all_expressions = Union[Symbol, Expression, Point3, Vector3, RotationMatrix, TransMatrix, Quaternion]
all_expressions_float = Union[Symbol, Expression, Point3, Vector3, RotationMatrix, TransMatrix, float, Quaternion]
symbol_expr_float = Union[Symbol, Expression, float]
symbol_expr = Union[Symbol, Expression]

pi: float

class CompiledFunction:
    str_params: Sequence[str]
    fast_f: ca.Function
    shape: Tuple[int, int]
    buf: ca.FunctionBuffer
    f_eval: functools.partial
    out: np.ndarray

    def __init__(self, str_params: Sequence[str], fast_f: ca.Function, shape: Tuple[int, int]): ...

    def __call__(self, **kwargs) -> np.ndarray: ...

    def call2(self, filtered_args: Iterable[float]) -> np.ndarray: ...


class Symbol_:
    s: ca.SX

    @property
    def shape(self) -> Tuple[int, int]: ...

    def __getitem__(self, item: Union[int, Tuple[int, int]]) -> Expression: ...

    def __setitem__(self, key: Union[int, Tuple[int, int]], value: symbol_expr_float): ...

    def __len__(self) -> int: ...

    def free_symbols(self) -> List[ca.SX]: ...

    def evaluate(self) -> Union[float, np.ndarray]: ...

    def compile(self, parameters: Optional[List[Symbol]] = None) -> CompiledFunction: ...

    def __hash__(self) -> int: ...


class Symbol(Symbol_):
    def __init__(self, name: str): ...

    @overload
    def __add__(self, other: Point3) -> Point3: ...
    @overload
    def __add__(self, other: Vector3) -> Vector3: ...
    @overload
    def __add__(self, other: Union[Symbol, Expression, float, Quaternion]) -> Expression: ...
    def __radd__(self, other: float) -> Expression: ...

    @overload
    def __sub__(self, other: Point3) -> Point3: ...
    @overload
    def __sub__(self, other: Vector3) -> Vector3: ...
    @overload
    def __sub__(self, other: RotationMatrix) -> RotationMatrix: ...
    @overload
    def __sub__(self, other: TransMatrix) -> TransMatrix: ...
    @overload
    def __sub__(self, other: Union[Symbol, Expression, float, Quaternion]) -> Expression: ...
    def __rsub__(self, other: float) -> Expression: ...

    @overload
    def __mul__(self, other: Point3) -> Point3: ...
    @overload
    def __mul__(self, other: Vector3) -> Vector3: ...
    @overload
    def __mul__(self, other: RotationMatrix) -> RotationMatrix: ...
    @overload
    def __mul__(self, other: TransMatrix) -> TransMatrix: ...
    @overload
    def __mul__(self, other: Union[Symbol, Expression, float, Quaternion]) -> Expression: ...
    def __rmul__(self, other: float) -> Expression: ...

    @overload
    def __truediv__(self, other: Point3) -> Point3: ...
    @overload
    def __truediv__(self, other: Vector3) -> Vector3: ...
    @overload
    def __truediv__(self, other: RotationMatrix) -> RotationMatrix: ...
    @overload
    def __truediv__(self, other: TransMatrix) -> TransMatrix: ...
    @overload
    def __truediv__(self, other: Union[Symbol, Expression, float, Quaternion]) -> Expression: ...
    def __rtruediv__(self, other: float) -> Expression: ...

    def __lt__(self, other: all_expressions_float) -> Expression: ...
    def __le__(self, other: all_expressions_float) -> Expression: ...
    def __gt__(self, other: all_expressions_float) -> Expression: ...
    def __ge__(self, other: all_expressions_float) -> Expression: ...
    def __neg__(self) -> Expression: ...
    def __pow__(self, other: all_expressions_float) -> Expression: ...
    def __rpow__(self, other: all_expressions_float) -> Expression: ...
    def __eq__(self, other: object) -> bool: ...
    def __ne__(self, other: object) -> bool: ...


class Expression(Symbol_):
    def __init__(self, data: Optional[Union[Symbol,
                                            Expression,
                                            float,
                                            Iterable[symbol_expr_float],
                                            Iterable[Iterable[symbol_expr_float]],
                                            np.ndarray]] = None): ...

    @overload
    def __add__(self, other: Point3) -> Point3: ...
    @overload
    def __add__(self, other: Vector3) -> Vector3: ...
    @overload
    def __add__(self, other: Union[Symbol, Expression, float, TransMatrix, RotationMatrix, Quaternion]) -> Expression: ...
    def __radd__(self, other: float) -> Expression: ...

    @overload
    def __sub__(self, other: Point3) -> Point3: ...
    @overload
    def __sub__(self, other: Vector3) -> Vector3: ...
    @overload
    def __sub__(self, other: Union[Symbol, Expression, float, TransMatrix, RotationMatrix, Quaternion]) -> Expression: ...
    def __rsub__(self, other: float) -> Expression: ...

    @overload
    def __mul__(self, other: Point3) -> Point3: ...
    @overload
    def __mul__(self, other: Vector3) -> Vector3: ...
    @overload
    def __mul__(self, other: Union[Symbol, Expression, float, RotationMatrix, TransMatrix, Quaternion]) -> Expression: ...
    def __rmul__(self, other: float) -> Expression: ...

    @overload
    def __truediv__(self, other: Point3) -> Point3: ...
    @overload
    def __truediv__(self, other: Vector3) -> Vector3: ...
    @overload
    def __truediv__(self, other: Union[Symbol, Expression, float, RotationMatrix, TransMatrix, Quaternion]) -> Expression: ...
    def __rtruediv__(self, other: float) -> Expression: ...

    def __lt__(self, other: symbol_expr_float) -> Expression: ...
    def __le__(self, other: symbol_expr_float) -> Expression: ...
    def __gt__(self, other: symbol_expr_float) -> Expression: ...
    def __ge__(self, other: symbol_expr_float) -> Expression: ...
    def __neg__(self) -> Expression: ...
    def __pow__(self, other: symbol_expr_float) -> Expression: ...
    def __rpow__(self, other: symbol_expr_float) -> Expression: ...

    @property
    def T(self) -> Expression: ...

    def remove(self, rows: List[int], columns: List[int]): ...

    def dot(self, other: Expression) -> Expression: ...


class Point3(Symbol_):
    @property
    def x(self) -> Expression: ...
    @x.setter
    def x(self, value: symbol_expr_float): ...
    @property
    def y(self) -> Expression: ...
    @y.setter
    def y(self, value: symbol_expr_float): ...
    @property
    def z(self) -> Expression: ...
    @z.setter
    def z(self, value: symbol_expr_float): ...

    def __init__(self, data: Optional[Union[Expression, Point3, Vector3,
                                            geometry_msgs.Point, geometry_msgs.PointStamped,
                                            geometry_msgs.Vector3, geometry_msgs.Vector3Stamped,
                                            ca.SX,
                                            np.ndarray,
                                            Iterable[symbol_expr_float]]] = None): ...

    @classmethod
    def from_xyz(cls,
                 x: Optional[symbol_expr_float] = None,
                 y: Optional[symbol_expr_float] = None,
                 z: Optional[symbol_expr_float] = None): ...

    def norm(self) -> Expression: ...

    @overload
    def __add__(self, other: Vector3) -> Point3: ...
    @overload
    def __add__(self, other: Symbol) -> Point3: ...
    @overload
    def __add__(self, other: Expression) -> Point3: ...
    @overload
    def __add__(self, other: float) -> Point3: ...
    def __radd__(self, other: float) -> Point3: ...

    @overload
    def __sub__(self, other: Point3) -> Vector3: ...
    @overload
    def __sub__(self, other: Union[Symbol, Expression, Vector3]) -> Point3: ...
    @overload
    def __sub__(self, other: float) -> Point3: ...
    def __rsub__(self, other: float) -> Point3: ...

    @overload
    def __mul__(self, other: Symbol) -> Point3: ...
    @overload
    def __mul__(self, other: Expression) -> Point3: ...
    @overload
    def __mul__(self, other: float) -> Point3: ...
    def __rmul__(self, other: float) -> Point3: ...

    @overload
    def __pow__(self, other: Symbol) -> Point3: ...
    @overload
    def __pow__(self, other: Expression) -> Point3: ...
    @overload
    def __pow__(self, other: float) -> Point3: ...
    def __rpow__(self, other: float) -> Point3: ...

    @overload
    def __truediv__(self, other: Symbol) -> Point3: ...
    @overload
    def __truediv__(self, other: Expression) -> Point3: ...
    @overload
    def __truediv__(self, other: float) -> Point3: ...
    def __rtruediv__(self, other: float) -> Point3: ...

    def __neg__(self) -> Point3: ...

    @overload
    def dot(self, other: Point3) -> Expression: ...
    @overload
    def dot(self, other: Vector3) -> Expression: ...


class Vector3(Symbol_):
    reference_frame: Optional[PrefixName]

    @property
    def x(self) -> Expression: ...
    @x.setter
    def x(self, value: symbol_expr_float): ...
    @property
    def y(self) -> Expression: ...
    @y.setter
    def y(self, value: symbol_expr_float): ...
    @property
    def z(self) -> Expression: ...
    @z.setter
    def z(self, value: symbol_expr_float): ...

    def __init__(self, data: Optional[Union[Expression, Point3, Vector3,
                                            geometry_msgs.Point, geometry_msgs.PointStamped,
                                            geometry_msgs.Vector3, geometry_msgs.Vector3Stamped,
                                            ca.SX,
                                            np.ndarray,
                                            Iterable[symbol_expr_float]]] = None): ...

    @classmethod
    def from_xyz(cls,
                 x: Optional[symbol_expr_float] = None,
                 y: Optional[symbol_expr_float] = None,
                 z: Optional[symbol_expr_float] = None): ...

    def norm(self) -> Expression: ...

    def scale(self, a: symbol_expr_float): ...

    @overload
    def __add__(self, other: Point3) -> Point3: ...
    @overload
    def __add__(self, other: Vector3) -> Vector3: ...
    @overload
    def __add__(self, other: Symbol) -> Vector3: ...
    @overload
    def __add__(self, other: Expression) -> Vector3: ...
    @overload
    def __add__(self, other: float) -> Vector3: ...
    def __radd__(self, other: float) -> Vector3: ...

    @overload
    def __sub__(self, other: Vector3) -> Vector3: ...
    @overload
    def __sub__(self, other: Point3) -> Point3: ...
    @overload
    def __sub__(self, other: Symbol) -> Vector3: ...
    @overload
    def __sub__(self, other: Expression) -> Vector3: ...
    @overload
    def __sub__(self, other: float) -> Vector3: ...
    def __rsub__(self, other: float) -> Vector3: ...

    @overload
    def __mul__(self, other: Symbol) -> Vector3: ...
    @overload
    def __mul__(self, other: Expression) -> Vector3: ...
    @overload
    def __mul__(self, other: float) -> Vector3: ...
    def __rmul__(self, other: float) -> Vector3: ...

    @overload
    def __truediv__(self, other: Symbol) -> Vector3: ...
    @overload
    def __truediv__(self, other: Expression) -> Vector3: ...
    @overload
    def __truediv__(self, other: float) -> Vector3: ...
    def __rtruediv__(self, other: float) -> Vector3: ...

    def __neg__(self) -> Vector3: ...

    @overload
    def __pow__(self, other: Symbol) -> Vector3: ...
    @overload
    def __pow__(self, other: Expression) -> Vector3: ...
    @overload
    def __pow__(self, other: float) -> Vector3: ...
    def __rpow__(self, other: float) -> Vector3: ...

    @overload
    def dot(self, other: Point3) -> Expression: ...
    @overload
    def dot(self, other: Vector3) -> Expression: ...

    def cross(self, other: Vector3) -> Vector3: ...


class TransMatrix(Symbol_):
    def __init__(self, data: Optional[Union[TransMatrix,
                                            RotationMatrix,
                                            ca.SX,
                                            np.ndarray,
                                            Expression,
                                            geometry_msgs.Pose,
                                            geometry_msgs.PoseStamped,
                                            geometry_msgs.Quaternion,
                                            geometry_msgs.QuaternionStamped,
                                            Iterable[Iterable[symbol_expr_float]]]] = None): ...

    @overload
    def dot(self, other: Point3) -> Point3: ...
    @overload
    def dot(self, other: Vector3) -> Vector3: ...
    @overload
    def dot(self, other: RotationMatrix) -> TransMatrix: ...
    @overload
    def dot(self, other: TransMatrix) -> TransMatrix: ...

    def to_rotation(self) -> RotationMatrix: ...
    def to_position(self) -> Point3: ...
    def to_translation(self) -> TransMatrix: ...

    @classmethod
    def from_xyz_rpy(cls,
                     x: Optional[symbol_expr_float] = None,
                     y: Optional[symbol_expr_float] = None,
                     z: Optional[symbol_expr_float] = None,
                     roll: Optional[symbol_expr_float] = None,
                     pitch: Optional[symbol_expr_float] = None,
                     yaw: Optional[symbol_expr_float] = None) -> TransMatrix: ...
    @classmethod
    def from_point_rotation_matrix(cls,
                                   point: Optional[Point3] = None,
                                   rotation_matrix: Optional[RotationMatrix] = None) -> TransMatrix: ...

    def inverse(self) -> TransMatrix: ...


class RotationMatrix(Symbol_):
    def __init__(self, data: Optional[Union[TransMatrix,
                                            RotationMatrix,
                                            Expression,
                                            Quaternion,
                                            ca.SX,
                                            np.ndarray,
                                            geometry_msgs.Quaternion,
                                            geometry_msgs.QuaternionStamped,
                                            Iterable[Iterable[symbol_expr_float]]]] = None): ...
    @classmethod
    def __quaternion_to_rotation_matrix(cls, q) -> List[List[symbol_expr_float]]: ...

    @classmethod
    def from_axis_angle(cls, axis: Vector3, angle: symbol_expr_float) -> RotationMatrix: ...
    @classmethod
    def from_quaternion(cls, q: Quaternion) -> RotationMatrix: ...
    @classmethod
    @overload
    def from_vectors(cls, x: Vector3, y: Vector3) -> RotationMatrix: ...
    @classmethod
    @overload
    def from_vectors(cls, x: Vector3, z: Vector3) -> RotationMatrix: ...
    @classmethod
    @overload
    def from_vectors(cls, y: Vector3, z: Vector3) -> RotationMatrix: ...
    @classmethod
    @overload
    def from_vectors(cls, x: Optional[Vector3] = None, y: Optional[Vector3] = None,
                     z: Optional[Vector3] = None) -> RotationMatrix: ...

    @classmethod
    def from_rpy(cls,
                 roll: Optional[symbol_expr_float] = None,
                 pitch: Optional[symbol_expr_float] = None,
                 yaw: Optional[symbol_expr_float] = None) -> RotationMatrix: ...

    def to_axis_angle(self) -> Tuple[Vector3, Expression]: ...
    def to_angle(self, hint: Callable) -> Expression: ...
    def to_rpy(self) -> Tuple[Expression, Expression, Expression]: ...
    def to_quaternion(self) -> Quaternion: ...

    @overload
    def dot(self, other: Point3) -> Point3: ...
    @overload
    def dot(self, other: Vector3) -> Vector3: ...
    @overload
    def dot(self, other: RotationMatrix) -> RotationMatrix: ...
    @overload
    def dot(self, other: TransMatrix) -> TransMatrix: ...

    def normalize(self): ...
    def inverse(self) -> RotationMatrix: ...
    @property
    def T(self) -> RotationMatrix: ...


class Quaternion(Symbol_):
    @property
    def x(self) -> Expression: ...
    @x.setter
    def x(self, value: symbol_expr_float): ...
    @property
    def y(self) -> Expression: ...
    @y.setter
    def y(self, value: symbol_expr_float): ...
    @property
    def z(self) -> Expression: ...
    @z.setter
    def z(self, value: symbol_expr_float): ...
    @property
    def w(self) -> Expression: ...
    @w.setter
    def w(self, value: symbol_expr_float): ...

    def __init__(self, data: Optional[Union[Expression,
                                            Quaternion,
                                            ca.SX,
                                            geometry_msgs.Quaternion,
                                            geometry_msgs.QuaternionStamped,
                                            Tuple[symbol_expr_float,
                                                  symbol_expr_float,
                                                  symbol_expr_float,
                                                  symbol_expr_float]]] = None): ...

    @classmethod
    def from_xyzw(cls,
                  x: symbol_expr_float,
                  y: symbol_expr_float,
                  z: symbol_expr_float,
                  w: symbol_expr_float) -> Quaternion: ...

    @classmethod
    def from_axis_angle(cls, axis: Vector3, angle: symbol_expr_float) -> Quaternion: ...
    @classmethod
    def from_rpy(cls, roll: symbol_expr_float, pitch: symbol_expr_float, yaw: symbol_expr_float) -> Quaternion: ...
    @classmethod
    def from_rotation_matrix(cls, r: RotationMatrix) -> Quaternion: ...

    def conjugate(self) -> Quaternion: ...
    def multiply(self, q: Quaternion) -> Quaternion: ...
    def diff(self, q: Quaternion) -> Quaternion: ...
    def norm(self) -> Expression: ...
    def normalize(self): ...
    def to_axis_angle(self) -> Tuple[Vector3, Expression]: ...
    def to_rotation_matrix(self) -> RotationMatrix: ...
    def to_rpy(self) -> Tuple[Expression, Expression, Expression]: ...
    def dot(self, other: Quaternion) -> Expression: ...

    def __neg__(self) -> Quaternion: ...


def norm(v: Union[Vector3, Point3, Expression, Quaternion]) -> Expression: ...

@overload
def save_division(nominator: Vector3, denominator: symbol_expr_float, if_nan: Optional[Vector3] = None) -> Vector3: ...
@overload
def save_division(nominator: Point3, denominator: symbol_expr_float, if_nan: Optional[Point3] = None) -> Point3: ...
@overload
def save_division(nominator: symbol_expr_float, denominator: symbol_expr_float,
                  if_nan: Optional[symbol_expr_float] = None) -> Expression: ...

def diag(args: Union[List[symbol_expr_float], Expression]) -> Expression: ...

def jacobian(expressions: Union[symbol_expr, List[symbol_expr]],
             symbols: Iterable[Symbol], order: int = 1) -> Expression: ...

def equivalent(expression1: symbol_expr, expression2: symbol_expr) -> bool: ...

def var(variables_names: str) -> List[Symbol]: ...

def free_symbols(expression: all_expressions) -> List[ca.SX]: ...

def create_symbols(names: List[str]) -> List[Symbol]: ...

def compile_and_execute(f: Callable[[Any], all_expressions],
                        params: Union[List[Union[float, np.ndarray]], np.ndarray]) \
        -> Union[float, np.ndarray]: ...

def zeros(x: int, y: int) -> Expression: ...

def ones(x: int, y: int) -> Expression: ...

def eye(size: int) -> Expression: ...

@overload
def abs(x: Vector3) -> Vector3: ...
@overload
def abs(x: Point3) -> Point3: ...
@overload
def abs(x: symbol_expr_float) -> Expression: ...

@overload
def max(x: Point3) -> Expression: ...
@overload
def max(x: Vector3) -> Expression: ...
@overload
def max(x: symbol_expr_float, y: Optional[symbol_expr_float] = None) -> Expression: ...

@overload
def min(x: Point3) -> Expression: ...
@overload
def min(x: Vector3) -> Expression: ...
@overload
def min(x: symbol_expr_float, y: Optional[symbol_expr_float] = None) -> Expression: ...

def limit(x: symbol_expr_float,
          lower_limit: symbol_expr_float,
          upper_limit: symbol_expr_float) -> Expression: ...

def equal(x: symbol_expr_float, y: symbol_expr_float) -> Expression: ...

def logic_and(*args: symbol_expr_float) -> symbol_expr_float: ...

def logic_or(*args: symbol_expr_float) -> symbol_expr_float: ...

@overload
def if_else(condition: symbol_expr_float, if_result: Vector3, else_result: Vector3) -> Vector3: ...
@overload
def if_else(condition: symbol_expr_float, if_result: Point3, else_result: Point3) -> Point3: ...
@overload
def if_else(condition: symbol_expr_float, if_result: symbol_expr_float,
            else_result: symbol_expr_float) -> Expression: ...

@overload
def if_greater(a: symbol_expr_float, b: symbol_expr_float, if_result: Vector3,
               else_result: Vector3) -> Vector3: ...
@overload
def if_greater(a: symbol_expr_float, b: symbol_expr_float, if_result: Point3,
               else_result: Point3) -> Point3: ...
@overload
def if_greater(a: symbol_expr_float, b: symbol_expr_float, if_result: symbol_expr_float,
               else_result: symbol_expr_float) -> Expression: ...

@overload
def if_less(a: symbol_expr_float, b: symbol_expr_float, if_result: Vector3,
            else_result: Vector3) -> Vector3: ...
@overload
def if_less(a: symbol_expr_float, b: symbol_expr_float, if_result: Point3,
            else_result: Point3) -> Point3: ...
@overload
def if_less(a: symbol_expr_float, b: symbol_expr_float, if_result: symbol_expr_float,
            else_result: symbol_expr_float) -> Expression: ...

@overload
def if_greater_zero(condition: symbol_expr_float,
                    if_result: Vector3,
                    else_result: Vector3) -> Vector3: ...
@overload
def if_greater_zero(condition: symbol_expr_float,
                    if_result: Point3,
                    else_result: Point3) -> Point3: ...
@overload
def if_greater_zero(condition: symbol_expr_float,
                    if_result: Quaternion,
                    else_result: Quaternion) -> Quaternion: ...
@overload
def if_greater_zero(condition: symbol_expr_float,
                    if_result: symbol_expr_float,
                    else_result: symbol_expr_float) -> Expression: ...

@overload
def if_greater_eq_zero(condition: symbol_expr_float, if_result: Vector3,
                       else_result: Vector3) -> Vector3: ...
@overload
def if_greater_eq_zero(condition: symbol_expr_float, if_result: Point3,
                       else_result: Point3) -> Point3: ...
@overload
def if_greater_eq_zero(condition: symbol_expr_float, if_result: symbol_expr_float,
                       else_result: symbol_expr_float) -> Expression: ...

@overload
def if_greater_eq(a: symbol_expr_float, b: symbol_expr_float, if_result: Vector3,
                  else_result: Vector3) -> Vector3: ...
@overload
def if_greater_eq(a: symbol_expr_float, b: symbol_expr_float, if_result: Point3,
                  else_result: Point3) -> Point3: ...
@overload
def if_greater_eq(a: symbol_expr_float, b: symbol_expr_float, if_result: symbol_expr_float,
                  else_result: symbol_expr_float) -> Expression: ...

@overload
def if_less_eq(a: symbol_expr_float, b: symbol_expr_float, if_result: Vector3,
               else_result: Vector3) -> Vector3: ...
@overload
def if_less_eq(a: symbol_expr_float, b: symbol_expr_float, if_result: Point3,
               else_result: Point3) -> Point3: ...
@overload
def if_less_eq(a: symbol_expr_float, b: symbol_expr_float, if_result: symbol_expr_float,
               else_result: symbol_expr_float) -> Expression: ...

@overload
def if_eq_zero(condition: symbol_expr_float,
               if_result: Vector3,
               else_result: Vector3) -> Vector3: ...
@overload
def if_eq_zero(condition: symbol_expr_float,
               if_result: Point3,
               else_result: Point3) -> Point3: ...
@overload
def if_eq_zero(condition: symbol_expr_float,
               if_result: symbol_expr_float,
               else_result: symbol_expr_float) -> Expression: ...

@overload
def if_eq(a: symbol_expr_float,
          b: symbol_expr_float,
          if_result: Vector3,
          else_result: Vector3) -> Vector3: ...
@overload
def if_eq(a: symbol_expr_float,
          b: symbol_expr_float,
          if_result: Point3,
          else_result: Point3) -> Point3: ...
@overload
def if_eq(a: symbol_expr_float,
          b: symbol_expr_float,
          if_result: symbol_expr_float,
          else_result: symbol_expr_float) -> Expression: ...

def if_eq_cases(a: symbol_expr_float,
                b_result_cases: Expression,
                else_result: symbol_expr_float) -> Expression: ...

def if_less_eq_cases(a: symbol_expr_float,
                     b_result_cases: Union[Expression, Sequence[Tuple[symbol_expr_float, symbol_expr_float]]],
                     else_result: symbol_expr_float) -> Expression: ...

def cross(u: Union[Vector3, Expression], v: Union[Vector3, Expression]) -> Vector3: ...

@overload
def scale(v: Vector3, a: symbol_expr_float) -> Vector3: ...
@overload
def scale(v: Point3, a: symbol_expr_float) -> Point3: ...
@overload
def scale(v: Expression, a: symbol_expr_float) -> Expression: ...

@overload
def dot(e1: TransMatrix, e2: Point3) -> Point3: ...
@overload
def dot(e1: TransMatrix, e2: Vector3) -> Vector3: ...
@overload
def dot(e1: RotationMatrix, e2: Point3) -> Point3: ...
@overload
def dot(e1: RotationMatrix, e2: Vector3) -> Vector3: ...
@overload
def dot(e1: RotationMatrix, e2: RotationMatrix) -> RotationMatrix: ...
@overload
def dot(e1: RotationMatrix, e2: TransMatrix) -> TransMatrix: ...
@overload
def dot(e1: TransMatrix, e2: RotationMatrix) -> TransMatrix: ...
@overload
def dot(e1: TransMatrix, e2: TransMatrix) -> TransMatrix: ...
@overload
def dot(e1: Quaternion, e2: Quaternion) -> Expression: ...
@overload
def dot(e1: Union[Vector3, Point3], e2: Union[Vector3, Point3]) -> Expression: ...
@overload
def dot(e1: Expression, e2: Expression) -> Expression: ...

def kron(m1: Expression, m2: Expression) -> Expression: ...

def trace(matrix: Union[Expression, RotationMatrix, TransMatrix]) -> Expression: ...

# def rotation_distance(a_R_b: Expression, a_R_c: Expression) -> Expression: ...

@overload
def vstack(list_of_matrices: List[Union[Point3, Vector3, Quaternion]]) -> Expression: ...
@overload
def vstack(list_of_matrices: List[TransMatrix]) -> Expression: ...
@overload
def vstack(list_of_matrices: List[Expression]) -> Expression: ...

@overload
def hstack(list_of_matrices: List[TransMatrix]) -> Expression: ...
@overload
def hstack(list_of_matrices: List[Expression]) -> Expression: ...

def normalize_axis_angle(axis: Vector3, angle: symbol_expr_float) -> Tuple[Vector3, Expression]: ...

def axis_angle_from_rpy(roll: symbol_expr_float, pitch: symbol_expr_float, yaw: symbol_expr_float) \
        -> Tuple[Vector3, Expression]: ...

def cosine_distance(v0: symbol_expr_float, v1: symbol_expr_float) -> Expression: ...

def euclidean_distance(v1: symbol_expr_float, v2: symbol_expr_float) -> Expression: ...

def fmod(a: symbol_expr_float, b: symbol_expr_float) -> Expression: ...

def normalize_angle_positive(angle: symbol_expr_float) -> Expression: ...

def normalize_angle(angle: symbol_expr_float) -> Expression: ...

def shortest_angular_distance(from_angle: symbol_expr_float, to_angle: symbol_expr_float) -> Expression: ...

def quaternion_slerp(q1: Quaternion, q2: Quaternion, t: symbol_expr_float) -> Quaternion: ...

@overload
def slerp(v1: Vector3, v2: Vector3, t: symbol_expr_float) -> Vector3: ...
@overload
def slerp(v1: Expression, v2: Expression, t: symbol_expr_float) -> Expression: ...

def save_acos(angle: symbol_expr_float) -> Expression: ...

def entrywise_product(matrix1: Expression, matrix2: Expression) -> Expression: ...

def floor(x: symbol_expr_float) -> Expression: ...

def ceil(x: symbol_expr_float) -> Expression: ...

def round_up(x: symbol_expr_float, decimal_places: symbol_expr_float) -> Expression: ...

def round_down(x: symbol_expr_float, decimal_places: symbol_expr_float) -> Expression: ...

def sum(matrix: Expression) -> Expression: ...

def sum_row(matrix: Expression) -> Expression: ...

def sum_column(matrix: Expression) -> Expression: ...

def distance_point_to_line_segment(point: Point3, line_start: Point3, line_end: Point3) \
        -> Tuple[Expression, Point3]: ...

def angle_between_vector(v1: Vector3, v2: Vector3) -> Expression: ...

def velocity_limit_from_position_limit(acceleration_limit: Union[Symbol, float],
                                       position_limit: Union[Symbol, float],
                                       current_position: Union[Symbol, float],
                                       step_size: Union[Symbol, float],
                                       eps: float = 1e-5) -> Expression: ...

def to_str(expression: all_expressions) -> str: ...

def total_derivative(expr: Union[Symbol, Expression],
                     symbols: Iterable[Symbol],
                     symbols_dot: Iterable[Symbol]) \
        -> Expression: ...

def quaternion_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion: ...

def quaternion_conjugate(q: Quaternion) -> Quaternion: ...

def quaternion_diff(q1: Quaternion, q2: Quaternion) -> Quaternion: ...

def sign(x: symbol_expr_float) -> Expression: ...

def cos(x: symbol_expr_float) -> Expression: ...

def sin(x: symbol_expr_float) -> Expression: ...

def sqrt(x: symbol_expr_float) -> Expression: ...

def acos(x: symbol_expr_float) -> Expression: ...

def atan2(x: symbol_expr_float, y: symbol_expr_float) -> Expression: ...


