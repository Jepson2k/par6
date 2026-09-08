"""Repeat an assumed offline observation scenario with the packaged model."""

from par6.client.dry_run_client import DryRunRobotClient


def main() -> None:
    client = DryRunRobotClient()
    client.delay(1.0)
    scenario = {
        "seed": 73,
        "observation_delay_s": 0.012,
        "encoder_noise_ticks": 20,
    }
    first = client.simulate(2.0, scenario=scenario)
    repeated = client.simulate(2.0, scenario=scenario)
    assert first.stop == "completed", first.blocks
    assert first.digest == repeated.digest
    print(first.stop, first.rows, first.digest.hex())


if __name__ == "__main__":
    main()
