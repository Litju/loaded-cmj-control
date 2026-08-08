"""Typed exceptions shared by rollout and isolated policy execution."""

from __future__ import annotations


class RuntimeErrorBase(Exception):
    """Base class for trusted runtime failures."""


class InvalidPolicyError(RuntimeErrorBase):
    """A policy artifact or policy response is invalid."""


class InvalidActionError(InvalidPolicyError):
    """The submitted policy returned an invalid action."""


class PolicyTimeoutError(TimeoutError, InvalidPolicyError):
    """The submitted policy exceeded its allowed response time."""


class PolicyProtocolError(InvalidPolicyError):
    """The submitted policy violated the request/response protocol."""


class InternalEvaluationError(RuntimeErrorBase):
    """Trusted runtime infrastructure failed while evaluating an episode."""


class InvalidNumericValue(InternalEvaluationError, ValueError):
    """Trusted runtime code produced a non-finite or otherwise invalid number."""

    def __init__(self, *, field: str, value_repr: str, message: str | None = None) -> None:
        detail = message or "expected a finite numeric value"
        super().__init__(f"{field}: {detail}; got {value_repr}")
        self.field = field
        self.value_repr = value_repr


class InvalidRuntimeContract(InternalEvaluationError):
    """A public runtime contract is internally inconsistent."""


__all__ = [
    "RuntimeErrorBase",
    "InternalEvaluationError",
    "InvalidActionError",
    "InvalidNumericValue",
    "InvalidPolicyError",
    "InvalidRuntimeContract",
    "PolicyProtocolError",
    "PolicyTimeoutError",
]
