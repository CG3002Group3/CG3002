
"""
Train walk or run based on raw values
"""

import numpy as np
from numpy import genfromtxt
import csv
from sklearn.ensemble import RandomForestClassifier
from sklearn.externals import joblib


def main():
    """Orchestrate the retrival of data, training and testing."""
    print("getting data")
    data = get_data()

    # Get classifier
    # from sklearn.svm import SVC
    # clf = SVC(probability=False,  # cache_size=200,
    #           kernel="rbf", C=2.8, gamma=.0073)
    clf = RandomForestClassifier(n_estimators=128, max_features="auto")
    print("Start fitting. This may take a while")

    # take all of it - make that number lower for experiments
    examples = len(data['train']['X'])
    clf.fit(data['train']['X'][:examples], data['train']['y'][:examples])

    analyze(clf, data)

    # Save model in a file
    joblib.dump(clf, 'v0.3.pkl')

def analyze(clf, data):
    """
    Analyze how well a classifier performs on data.

    Parameters
    ----------
    clf : classifier object
    data : dict
    """
    # Get confusion matrix
    from sklearn import metrics
    predicted = clf.predict(data['test']['X'])
    print("Confusion matrix:\n%s" %
          metrics.confusion_matrix(data['test']['y'],
                                   predicted))
    print("Accuracy: %0.4f" % metrics.accuracy_score(data['test']['y'],
                                                     predicted))

def get_data():

    from sklearn.utils import shuffle
    y = []
    x = []
    z = []
    # Get data from json file
    with open('all2.csv', 'rb') as csvfile:
        acc_reader = csv.reader(csvfile, delimiter=',')
        i = 0
        for row in acc_reader:
            z.append([float(number) for number in row[0:13]])

        # Extract Time Domain Features
        interval_data = []
        accum = []
        j = 0
        for reading in z:
            accum.append(reading)
            if j == 19:
                interval_data.append(accum)
                accum = []
                j = 0
            else:
                j += 1

        for single_interval in interval_data:
            numpy_interval = np.array(single_interval)
            mean = np.mean(numpy_interval, axis=0)
            variance = np.var(numpy_interval, axis=0)
            # Overkill with more features
            median = np.median(numpy_interval, axis=0)
            if (mean[12] == 1 or mean[12] == 2 or mean[12] == 3 or mean[12] == 4 or mean[12] == 5 or mean[12] == 6):
                x.append(np.append(mean[0:12], [variance[0:12], median[0:12]]).tolist())
                y.append(mean[12])

        # Create data for training  
        y = np.array(y)
        x = np.array(x)
        print(x.shape, y.shape)
        x, y = shuffle(x, y, random_state=0)

        from sklearn.cross_validation import train_test_split
        x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2)
        data = {'train': {'X': x_train,
                        'y': y_train},
                'test': {'X': x_test,
                        'y': y_test}}
    return data


if __name__ == '__main__':
    main()