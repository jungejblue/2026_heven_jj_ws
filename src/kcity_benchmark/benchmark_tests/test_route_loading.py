from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from kcity_benchmark.route_tracker import load_route_reference
from kcity_scenario_manager.route_csv import write_route_csv


def rows():
    result=[]
    for index,(route_s,x) in enumerate(((0.0,0.0),(1.0,1.0),(2.0,2.0))):
        result.append({
            'index':index,'route_s_m':route_s,'x':x,'y':0.0,'z':0.0,
            'yaw':0.0,'road_id':1,'section_id':0,'lane_id':-1,
            'opendrive_s':route_s,'is_junction':False,
        })
    return result


class RouteLoadingTests(unittest.TestCase):
    def test_route_csv_has_priority_and_metadata(self):
        with TemporaryDirectory() as directory:
            package=Path(directory)/'package'
            config_dir=package/'config'
            config_dir.mkdir(parents=True)
            config_file=config_dir/'qualifier.yaml'
            config_file.write_text('benchmark: {}\n',encoding='utf-8')
            route_path=package/'routes/qualifier.csv'
            write_route_csv(route_path,rows())
            points,metadata=load_route_reference(config_file,{
                'route_csv':'routes/qualifier.csv',
                'route_points':[[99,99],[100,100]],
            })
            self.assertEqual(points,[(0.0,0.0),(1.0,0.0),(2.0,0.0)])
            self.assertEqual(metadata['route_source'],'route_csv')
            self.assertEqual(metadata['route_csv'],'routes/qualifier.csv')
            self.assertEqual(metadata['route_point_count'],3)
            self.assertEqual(metadata['route_length_m'],2.0)
            self.assertEqual(len(metadata['route_sha256']),64)

    def test_route_points_backward_compatibility(self):
        points,metadata=load_route_reference('/tmp/config.yaml',{
            'route_points':[[0,0],[3,4]],
        })
        self.assertEqual(points,[(0.0,0.0),(3.0,4.0)])
        self.assertEqual(metadata,{
            'route_source':'route_points',
            'route_csv':None,
            'route_point_count':2,
            'route_length_m':5.0,
            'route_sha256':None,
        })


if __name__=='__main__':
    unittest.main()
