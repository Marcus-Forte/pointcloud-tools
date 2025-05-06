from learning3d.models import Classifier, PointNet, Segmentation


def main():
    classifier = Classifier(feature_model=PointNet(), num_classes=40)
    seg = Segmentation(feature_model=PointNet(), num_classes=40)

    print(seg)

if __name__ == "__main__":
    main()