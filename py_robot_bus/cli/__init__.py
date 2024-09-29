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


@cli.command(name="multiplex")
def multiplex():
    click.echo('All simulation')
    from multiplex import Multiplex

    mp = Multiplex()
    mp.add(f"prb pad_control")
    mp.add(f"prb engine")
    mp.add(f"prb matplot")
    mp.add(f"prb pybullet")
    mp.run()


@cli.command(name="engine")
def engine():
    click.echo('Engine')
    main_engine()


@cli.command(name="pybullet")
def pybullet():
    click.echo('pybullet simulation')
    main_pybullet()


@cli.command(name="matplot")
def matplot():
    click.echo('Matplot visualizer')
    main_matplot()


@cli.command(name="pad_control")
def pad_control():
    click.echo('Pad Control listener')
    main_pad_control()


if __name__ == '__main__':
    cli()