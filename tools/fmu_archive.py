"""Bounded ZIP extraction into an empty, privately owned directory; no shell."""
import pathlib
import shutil
import stat
import sys
import zipfile


def extract(source, destination):
    root = pathlib.Path(destination)
    if any(root.iterdir()):
        raise ValueError("FMU destination must be empty")
    with zipfile.ZipFile(source) as archive:
        entries = archive.infolist()
        if len(entries) > 10000 or sum(e.file_size for e in entries) > 512 * 1024 * 1024:
            raise ValueError("FMU archive exceeds extraction limits")
        names = set()
        for entry in entries:
            path = pathlib.PurePosixPath(entry.filename)
            mode = entry.external_attr >> 16
            if (path.is_absolute() or '..' in path.parts or '\\' in entry.filename
                    or ':' in entry.filename or not path.parts
                    or (stat.S_IFMT(mode) not in (0, stat.S_IFREG, stat.S_IFDIR))):
                raise ValueError(f"Unsafe FMU entry: {entry.filename}")
            normalized = str(path)
            if normalized in names:
                raise ValueError(f"Duplicate FMU entry: {entry.filename}")
            names.add(normalized)
        for entry in entries:
            target = root / entry.filename
            if entry.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(entry) as src, target.open('xb') as dst:
                    shutil.copyfileobj(src, dst)


if __name__ == '__main__':
    try:
        extract(sys.argv[1], sys.argv[2])
    except Exception as error:
        sys.exit(f"FMU extraction failed: {error}")
