import base64
import json
from pathlib import Path
import re
import struct
import tempfile
import unittest
import zlib
import numpy as np
from field_demo.preview import png_bytes,pixel_xyz,render_preview


class PreviewTests(unittest.TestCase):
    def test_png_signature_dimensions_crc_and_exact_pixel_rows(self):
        image=np.array([[[255,0,0],[0,255,0]],[[0,0,255],[1,2,3]]],dtype=np.uint8)
        png=png_bytes(image)
        self.assertEqual(png[:8],b'\x89PNG\r\n\x1a\n')
        self.assertEqual(struct.unpack('>II',png[16:24]),(2,2))
        pos=8;data=b''
        while pos<len(png):
            length=struct.unpack('>I',png[pos:pos+4])[0]
            kind,chunk=png[pos+4:pos+8],png[pos+8:pos+8+length]
            self.assertEqual(struct.unpack('>I',png[pos+8+length:pos+12+length])[0],zlib.crc32(kind+chunk)&0xffffffff)
            if kind==b'IDAT':data+=chunk
            pos+=12+length
        self.assertEqual(zlib.decompress(data),b'\x00'+image[0].tobytes()+b'\x00'+image[1].tobytes())

    def test_pixel_xyz_keeps_sdk_values_rejects_zero_nan_and_outside(self):
        points=np.array([[[.123,.456,.789],[0,0,0]],[[np.nan,0,1],[1,2,3]]],dtype=np.float32)
        np.testing.assert_allclose(pixel_xyz(points,0,0),[.123,.456,.789])
        for x,y in ((1,0),(0,1),(2,0),(-1,0)):
            with self.assertRaises(ValueError):pixel_xyz(points,x,y)

    def test_standalone_html_embeds_exact_sdk_cloud_safely_no_network_assets(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'sample.npz';output=root/'preview.html'
            points=np.array([[[.123,.456,.789],[0,0,0]],[[np.nan,0,1],[1,2,3]]],dtype=np.float32)
            np.savez(source,rgb=np.zeros((2,2,3),dtype=np.uint8),points_camera_image_m=points)
            source.with_suffix('.json').write_text(json.dumps(dict(camera_frame='</script><script>alert(1)</script>',camera_serial='TEST_ONLY')))
            r=render_preview(source,output);text=output.read_text()
            self.assertTrue(r['pixel_xyz_available'])
            payload=json.loads(re.search(r'<script id="payload" type="application/json">(.*?)</script>',text).group(1))
            actual=np.frombuffer(base64.b64decode(payload['points_base64']),dtype='<f4').reshape(2,2,3)
            np.testing.assert_equal(actual,points)
            self.assertNotIn('</script><script>alert',text)
            for forbidden in ('http://','https://','fetch(','WebSocket','cdn.'):
                self.assertNotIn(forbidden,text)
            self.assertIn('No valid measured depth',text)

    def test_old_flat_snapshot_displays_rgb_without_inventing_pixel_xyz(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'old.npz';output=root/'old.html'
            np.savez(source,rgb=np.zeros((2,2,3),dtype=np.uint8),points_camera_m=np.array([[1,2,3]]))
            report=render_preview(source,output)
            self.assertFalse(report['pixel_xyz_available'])
            self.assertIn('No coordinate will be invented',output.read_text())


if __name__=='__main__':unittest.main()
