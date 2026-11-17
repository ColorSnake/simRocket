import pathlib
import stat
import tempfile
import unittest
import zipfile
from fmu_archive import extract

class ArchiveTests(unittest.TestCase):
    def test_rejects_traversal_and_symlink(self):
        for name, mode in [('../outside', 0), ('/absolute', 0), ('a/../../outside', 0), ('link', stat.S_IFLNK | 0o777)]:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                root=pathlib.Path(tmp); archive=root/'test.fmu'; dest=root/'dest'; dest.mkdir()
                with zipfile.ZipFile(archive,'w') as z:
                    entry=zipfile.ZipInfo(name); entry.external_attr=mode<<16; z.writestr(entry,'target')
                with self.assertRaises(ValueError): extract(archive,dest)
                self.assertEqual(list(dest.iterdir()),[])
    def test_shell_characters_are_literal(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp); archive=root/'model $(not-a-command).fmu'; dest=root/'dest'; dest.mkdir()
            with zipfile.ZipFile(archive,'w') as z: z.writestr('modelDescription.xml','test')
            extract(archive,dest)
            self.assertEqual((dest/'modelDescription.xml').read_text(),'test')
    def test_rejects_duplicate_entries(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp); archive=root/'test.fmu'; dest=root/'dest'; dest.mkdir()
            with zipfile.ZipFile(archive,'w') as z:
                z.writestr('a/b','one'); z.writestr('a/./b','two')
            with self.assertRaises(ValueError): extract(archive,dest)

if __name__=='__main__': unittest.main()
