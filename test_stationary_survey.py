import unittest
from stationary_survey import StationarySurvey


class SurveyTests(unittest.TestCase):
    def test_elapsed_settling_alone_does_not_allow_motion(self):
        survey = StationarySurvey(1., 3)
        survey.reset(100.)
        self.assertFalse(survey.ready(105.))

    def test_requires_scans_and_matching_pose_and_dwell(self):
        survey = StationarySurvey(1., 3)
        survey.reset(100.)
        for stamp in (200., 200.1, 200.2):
            survey.observe(stamp)
        survey.localize(200.1)
        self.assertFalse(survey.ready(102.))
        survey.localize(200.2)
        self.assertFalse(survey.ready(100.5))
        self.assertTrue(survey.ready(101.))

    def test_old_pose_and_duplicate_scans_cannot_complete_next_stop(self):
        survey = StationarySurvey(1., 3)
        survey.reset(100.)
        for stamp in (200., 200., 199., float('nan')):
            survey.observe(stamp)
        self.assertEqual(survey.count, 1)
        survey.reset(110.)
        for stamp in (210., 210.1, 210.2):
            survey.observe(stamp)
        survey.localize(200.)
        self.assertFalse(survey.ready(115.))


if __name__ == '__main__':
    unittest.main()
