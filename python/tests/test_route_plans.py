"""A-GRA's route plans through Python (docs/flight-autonomy.md, 4.39): prepared for upload, published, uploaded and read back
with their planning metadata; prepared for activation and activated; superseded; deactivated before they execute and not
after; FA's own deactivation; a plan for planning use only never activated; A-GRA's names for their states."""
import math
import unittest

import fsim
from fsim import agra

R = 6371008.8  # the platform's mean radius


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class RoutePlansTest(unittest.TestCase):
    def setUp(self):
        self.w = make_world("py-route-plans")
        self.v = self.w.create_vehicle("planned", "jsbsim:c172x", latitude_deg=37.6188, longitude_deg=-122.375, altitude_msl_m=1500.0,
                                       airspeed_ms=55.0, heading_deg=90.0)
        self.w.step(10)
        lat, lon = self.v.state.latitude_rad, self.v.state.longitude_rad
        self.points = [fsim.Waypoint(lat, lon + east / (R * math.cos(lat)), 1500.0) for east in (3000.0, 6000.0)]

    def plan(self, plan_id, points=None, **fields):
        return fsim.RoutePlan(plan_id, fsim.BatchCommand("submit_route", points or self.points), **fields)

    def upload(self, plan):
        self.assertTrue(self.v.plan_command(plan.id, "prepare_for_upload").completed)
        self.v.publish_plan(plan)
        self.assertTrue(self.v.plan_command(plan.id, "upload").completed)

    def test_uploaded_and_read_back(self):
        v = self.v
        p = self.plan(7, version=3, detailed=True, remarks_name="east", remarks="two points east",
                      point_metadata=[fsim.PointMetadata(0, "operator_defined", locked=True, remarks="the first", fix_key="ALPHA",
                                                         fix_system="DAFIF"),
                                      fsim.PointMetadata(1, modified=True)],
                      path_metadata=[fsim.PathMetadata(0, fsim.RouteState(altitude_m=1500.0), fuel_kg=80.0, transition_plan=9)])
        with self.assertRaises(fsim.Rejected) as refused:
            v.publish_plan(p)
        self.assertEqual(refused.exception.reason, "wrong_plan_state")  # (FA listens for those prepared for upload alone)
        r = v.plan_command(7, "prepare_for_upload")
        self.assertEqual((r.completed, r.state, r.plan, r.command), (True, fsim.PlanState.READY_FOR_UPLOAD, 7, fsim.PlanCommand.PREPARE_FOR_UPLOAD))
        with self.assertRaises(fsim.Rejected) as refused:
            v.publish_plan(p._replace(remarks="tab\there"))
        self.assertEqual(refused.exception.reason, "invalid_parameter")
        v.publish_plan(p)
        r = v.plan_command(7, fsim.PlanCommand.UPLOAD)
        self.assertEqual((r.completed, r.state), (True, fsim.PlanState.UPLOADED))
        s = v.plan_status(7)
        self.assertEqual((s.version, s.revision, s.execution, s.reason), (3, 1, fsim.PlanExecution.NONE, "none"))
        self.assertEqual([t.id for t in v.plans()], [7])
        # read back as given (A-GRA's query for the plan)
        back = v.plan(7)
        self.assertEqual((back.id, back.version, back.detailed, back.remarks_name, back.remarks), (7, 3, True, "east", "two points east"))
        self.assertEqual(back.route.method, "submit_route")
        self.assertEqual([q.altitude_m for q in back.route.args[0]], [1500.0, 1500.0])
        first = back.point_metadata[0]
        self.assertEqual((first.point, first.source, first.locked, first.modified, first.remarks, first.fix_key, first.fix_system),
                         (0, fsim.PointSource.OPERATOR_DEFINED, True, False, "the first", "ALPHA", "DAFIF"))
        self.assertEqual((back.point_metadata[1].modified, back.point_metadata[1].source), (True, fsim.PointSource.AUTO_ROUTED))
        path = back.path_metadata[0]
        self.assertEqual((path.path, path.initial.altitude_m, path.fuel_kg, path.transition_plan), (0, 1500.0, 80.0, 9))
        self.assertTrue(math.isnan(path.endurance_s) and math.isnan(path.initial.time_s))
        # an upload with nothing received fails: an answer, not an error; A-GRA's names for it
        v.plan_command(8, "prepare_for_upload")
        r = v.plan_command(8, "upload")
        self.assertEqual((r.completed, r.state, r.reason), (False, fsim.PlanState.UPLOAD_FAILED, "plan_not_received"))
        self.assertEqual(agra.plan_activation_status(r), {"CommandType": "UPLOAD", "PlanActivationCommandState": "UPLOAD_FAILED",
                                                          "CommandStatus": "FAILED", "Reason": "INIT_CRITERIA_NOT_MET"})
        self.assertIsNone(v.plan(8))

    def test_activated_superseded_deactivated(self):
        v, w = self.v, self.w
        self.upload(self.plan(21))
        # a point it cannot fly: its preparation for activation fails, naming it
        upside_down = self.points[1]._replace(altitude_min_m=2000.0, altitude_max_m=1800.0)
        self.upload(self.plan(22, [self.points[0], upside_down]))
        r = v.plan_command(22, "prepare_for_activation")
        self.assertEqual((r.completed, r.state, r.reason, r.index),
                         (False, fsim.PlanState.PREPARATION_FOR_ACTIVATION_FAILED, "invalid_waypoint", 1))
        direct = v.submit_route([self.points[0], upside_down], validate_only=True)  # (as its route's validation answers)
        self.assertEqual((direct.valid, direct.reason, len(direct.findings)), (False, "invalid_waypoint", len(r.findings)))
        r = v.plan_command(21, "prepare_for_activation")
        self.assertEqual((r.completed, r.state, r.findings), (True, fsim.PlanState.READY_FOR_ACTIVATION, []))
        r = v.plan_command(21, "activate", command_id=44)
        self.assertTrue(r.completed)
        self.assertIsInstance(r.activity, fsim.Activity)
        w.step(10)
        s = v.plan_status(21)
        self.assertEqual((s.state, s.execution, s.activity, s.command_id), (fsim.PlanState.ACTIVATED, fsim.PlanExecution.EXECUTING, r.activity.id, 44))
        self.assertEqual(agra.plan_execution_state(s), "EXECUTING")
        r = v.plan_command(21, "deactivate")
        self.assertEqual((r.completed, r.state, r.reason), (False, fsim.PlanState.ACTIVATED, "plan_executing"))
        # another command takes its axes: superseded (A-GRA's SUPERCEDED); taken back once done
        v.submit_hsa(heading_rad=0.0, speed=55.0, altitude_m=1500.0)
        s = v.plan_status(21)
        self.assertEqual((s.execution, s.reason), (fsim.PlanExecution.SUPERSEDED, "preempted"))
        self.assertEqual(agra.plan_execution_state(s), "SUPERCEDED")
        self.assertTrue(v.plan_command(21, "deactivate").completed)
        # FA's own deactivation, before it flies
        self.assertTrue(v.plan_command(21, "prepare_for_activation").completed)
        r = v.abort_plan(21, "collision_avoidance")
        self.assertEqual((r.completed, r.state), (True, fsim.PlanState.DEACTIVATED))
        s = v.plan_status(21)
        self.assertEqual((s.execution, s.reason), (fsim.PlanExecution.CANCELED, "collision_avoidance"))
        # for planning use only: never activated; forgotten
        self.upload(self.plan(23, for_planning_use_only=True))
        self.assertTrue(v.plan_status(23).for_planning_use_only)
        r = v.plan_command(23, "prepare_for_activation")
        self.assertEqual((r.completed, r.reason), (False, "planning_only"))
        v.remove_plan(23)
        self.assertIsNone(v.plan_status(23))
        with self.assertRaises(fsim.Rejected) as refused:
            v.remove_plan(23)
        self.assertEqual(refused.exception.reason, "unknown_plan")


    def test_airfields_and_fa_plans(self):
        """FA's airfields and its own plans (docs/flight-autonomy.md, 4.40): loaded by the platform, read back, read only to MA;
        MA's own for a landing refused."""
        v = self.v
        lat, lon = v.state.latitude_rad, v.state.longitude_rad
        place = lambda north: fsim.RunwayPoint(lat + north / R, lon, 12.0)  # noqa: E731
        two = fsim.Runway(2, 0.0, 2500.0, takeoff=fsim.RunwayCoordinates(place(20000.0), limit=place(22500.0)),
                          landing=fsim.RunwayCoordinates(place(20000.0), place(20300.0), place(22500.0)))
        field = fsim.Airfield(5, "KXYZ", 101325.0, [two])
        with self.assertRaises(fsim.Rejected) as refused:
            v.load_airfield(field._replace(icao="kxyz"))
        self.assertEqual(refused.exception.reason, "invalid_parameter")
        v.load_airfield(field)
        back = v.airfield(5)
        self.assertEqual((back.id, back.icao, back.qnh_pa, back.revision, len(back.runways)), (5, "KXYZ", 101325.0, 1, 1))
        self.assertEqual(back.runways[0].landing.threshold.altitude_m, 12.0)
        self.assertTrue(math.isnan(back.runways[0].takeoff.threshold.latitude_rad))
        self.assertEqual([a.id for a in v.airfields()], [5])
        self.assertIsNone(v.airfield(6))
        report = agra.airfield_report(back)
        self.assertEqual((report["AirfieldID"], report["Information"]["ICAO_Code"]), (5, "KXYZ"))
        self.assertIsNone(report["Information"]["Runway"][0]["TakeoffCoordinates"]["Threshold"])
        self.assertEqual(report["Information"]["Runway"][0]["LandingCoordinates"]["Limit"]["Altitude"], 12.0)
        # FA's landing plan: its landing's path names runway 2 of airfield 5
        route = fsim.BatchCommand("submit_route", self.points, paths=[fsim.RoutePath(1, "final_approach", 0, 1), fsim.RoutePath(2, "landing", 1, 1)])
        plan = fsim.RoutePlan(71, route, path_metadata=[fsim.PathMetadata(1, airfield=5, runway=9)])
        with self.assertRaises(fsim.Rejected) as refused:
            v.load_plan(plan)
        self.assertEqual(refused.exception.reason, "unknown_airfield")
        v.load_plan(plan._replace(path_metadata=[fsim.PathMetadata(1, airfield=5, runway=2)]))
        s = v.plan_status(71)
        self.assertEqual((s.state, s.fa_owned), (fsim.PlanState.UPLOADED, True))
        r = v.plan_command(71, "prepare_for_upload")
        self.assertEqual((r.completed, r.reason), (False, "read_only_plan"))
        kept = v.plan(71).path_metadata[0]
        self.assertEqual((kept.path, kept.airfield, kept.runway), (1, 5, 2))
        # MA's own for a landing: refused as published
        v.plan_command(72, "prepare_for_upload")
        with self.assertRaises(fsim.Rejected) as refused:
            v.publish_plan(plan._replace(id=72, path_metadata=()))
        self.assertEqual(refused.exception.reason, "safety_critical_plan")
        self.assertEqual(agra.CANNOT_COMPLY["read_only_plan"], "INELIGIBLE_CONTROL_SOURCE")


if __name__ == "__main__":
    unittest.main()
