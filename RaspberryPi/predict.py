from sklearn.externals import joblib
import numpy as np
import time

move = ["WTF", "Wavehand", "Jump", "Frontback", "Turnclap", "Window"]
clf = joblib.load('0.98_v2.pkl')

def predict_data(data_set):
    data_array = [row.split(',') for row in list(data_set)]
    data_array = [row[:-3] for row in data_array]
    data_array = [[float(number) for number in row] for row in data_array]

    # Features extraction
    numpy_data = np.array(data_array)
    mean = np.mean(numpy_data, axis=0)
    variance = np.var(numpy_data, axis=0)
    median = np.median(numpy_data, axis=0)

    x = np.append(mean[0:12], [variance[0:12], median[0:12]]).tolist()
    x = np.array(x).reshape(1,-1)
    prev_time = time.time()
    print(move[int(clf.predict(x)[0])] + " in " + str(time.time() - prev_time) + "s")

#predict_data(["1,2,3,4,5,3,1,2,3,4,5,3,10,11,13", "1,2,3,3,3,3,1,2,3,4,5,3,10,11,12"])
