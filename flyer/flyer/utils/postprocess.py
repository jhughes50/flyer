
from typing import List

def postprocess_labels(labels : List) -> List:
    return [l[:len(l)//2] for l in labels]
