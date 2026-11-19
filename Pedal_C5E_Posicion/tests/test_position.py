import unittest,sys,math
from dataclasses import asdict
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pedal_model import PedalPosicion,CalibracionBloom

class TestPosition(unittest.TestCase):
    def test_mode(self):
        self.assertEqual(PedalPosicion().driver.modo_cia402,1)
    def test_calibration(self):
        c=CalibracionBloom.cargar()
        self.assertAlmostEqual(c.demanda_pct_a_ticks(1),1900.,places=3)
        self.assertAlmostEqual(c.demanda_pct_a_ticks(10),4339.778182320678,places=4)
        self.assertAlmostEqual(c.desaceleracion_a_demanda_pct(.1),4.770365748,places=5)
    def test_tracking(self):
        for pct in (25,50,75,100):
            p=PedalPosicion.con_mm_por_6900(20)
            data=None
            for _ in range(250):data=p.step(pct,.02)
            self.assertLess(abs(data.position_pct-pct),.5, (pct,data.position_pct))
            self.assertEqual(data.control_mode,1)
            self.assertTrue(0<=data.rod_displacement_mm<=20)
    def test_retract(self):
        p=PedalPosicion()
        for _ in range(100):p.step(100,.02)
        for _ in range(150):out=p.step(0,.02)
        self.assertLess(out.rod_displacement_mm,.1)
    def test_limits(self):
        p=PedalPosicion()
        for _ in range(120):d=p.step_target_ticks(100000,.02)
        self.assertEqual(d.target_position_607A_ticks,6900)
        self.assertTrue(d.saturation_warning)
        self.assertLessEqual(d.rod_displacement_mm,20.001)
    def test_no_nan(self):
        p=PedalPosicion()
        for x in [0,25,100,0,75,0]:
            for _ in range(40):d=p.step(x,.01)
            for k,v in asdict(d).items():
                if isinstance(v,float):self.assertTrue(math.isfinite(v), k)

if __name__=='__main__':unittest.main()
