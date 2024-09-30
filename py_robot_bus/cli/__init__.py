import os

import click
from py_robot_bus.cli.engine import main as main_engine
from py_robot_bus.cli.pybullet import main as main_pybullet
from py_robot_bus.cli.matplot import main as main_matplot
from py_robot_bus.cli.pad_control import main as main_pad_control
import colored_traceback

try:
    colored_traceback.add_hook(always=True)
except:
    pass


@click.group()
def cli():
    pass


@cli.group()
def dev():
    pass


@cli.command(name="multiplex")
def multiplex():
    click.echo("All simulation")
    from multiplex import Multiplex

    mp = Multiplex()
    mp.add("prb pad-control")
    mp.add("prb engine")
    mp.add("prb matplot")
    mp.add("prb pybullet")
    mp.run()


@cli.command(name="engine")
def engine():
    click.echo("Engine")
    main_engine()


@cli.command(name="pybullet")
def pybullet():
    click.echo("pybullet simulation")
    main_pybullet()


@cli.command(name="matplot")
def matplot():
    click.echo("Matplot visualizer")
    main_matplot()


@cli.command(name="pad-control")
def pad_control():
    click.echo("Pad Control listener")
    main_pad_control()


@dev.command(name="dump-schema")
def pad_dump_schema():
    click.echo("dump-schema")
    from py_robot_bus.tools import dump_json_schema
    import py_robot_bus.bus.message
    import py_robot_bus

    dump_json_schema(
        py_robot_bus.bus.message,
        os.path.join(os.path.dirname(py_robot_bus.__file__), "..", "json_model"),
    )


@dev.command(name="bezier")
def bezier():
    click.echo("bezier")
    from py_robot_bus.io.visualizer.bezier_matplot import BezierVisualizer

    visualizer = BezierVisualizer()
    visualizer.run()


if __name__ == "__main__":
    cli()
