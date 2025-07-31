import pyvista as pv
from pyvistaqt import BackgroundPlotter

plotter = BackgroundPlotter()
plotter.add_mesh(pv.Sphere(), color='red')
plotter.show()