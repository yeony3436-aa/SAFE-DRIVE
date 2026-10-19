"""Pedestrian crossing that starts only on the published WALK indication."""

from dataclasses import dataclass

from safe_drive_scenario.sudden_pedestrian import ScenarioState


@dataclass
class SignalCrossing:
    start_y: float
    end_y: float
    speed: float

    def __post_init__(self) -> None:
        if self.speed <= 0.0 or self.end_y <= self.start_y:
            raise ValueError('crossing speed and endpoints must be positive')
        self.y = self.start_y
        self.state = ScenarioState.ARMED

    def step(self, signal: str | None, elapsed: float) -> ScenarioState:
        if elapsed < 0.0:
            raise ValueError('elapsed must be nonnegative')
        if self.state == ScenarioState.ARMED and signal == 'WALK':
            self.state = ScenarioState.ACTIVE
        if self.state == ScenarioState.ACTIVE:
            # Once entered on WALK, finish crossing during clearance time.
            self.y = min(self.end_y, self.y + self.speed * elapsed)
            if self.y >= self.end_y:
                self.state = ScenarioState.COMPLETED
        return self.state
