"""Long-lived interfaces between reasoning, coding, and robot execution.

The first rollout still sends every request to the existing agent provider.
These contracts keep future planner models from inheriting shell or robot
credentials merely because they can produce text.
"""

from __future__ import annotations

from typing import Any, AsyncIterator, Dict, List, Literal, Optional, Protocol

from pydantic import BaseModel, Field


class PlannerRequest(BaseModel):
    operator_prompt: str
    system_context: str
    selected_robot: Optional[str] = None


class PlannerDecision(BaseModel):
    action: Literal["respond", "diagnose", "repair", "mission", "ask"]
    target_robots: List[str] = Field(default_factory=list)
    instructions: str
    confidence: float = Field(ge=0.0, le=1.0)
    requires_approval: bool = True


class PlannerModel(Protocol):
    name: str

    def status(self) -> Dict[str, Any]: ...

    async def plan(self, request: PlannerRequest) -> PlannerDecision: ...


class ChangeRequest(BaseModel):
    instruction: str
    workspace: str
    context: Dict[str, Any] = Field(default_factory=dict)
    session_id: Optional[str] = None


class CodingWorker(Protocol):
    name: str

    def status(self) -> Dict[str, Any]: ...

    def run(self, request: ChangeRequest) -> AsyncIterator[Dict[str, Any]]: ...


class FleetAction(BaseModel):
    action: Literal["doctor", "deploy", "drive", "navigate", "cancel", "stop", "body"]
    robot_ids: List[str]
    parameters: Dict[str, Any] = Field(default_factory=dict)
    approved: bool = False


class FleetTools(Protocol):
    async def invoke(self, action: FleetAction) -> Dict[str, Any]: ...
