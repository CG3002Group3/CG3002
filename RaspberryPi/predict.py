from sklearn.externals import joblib
import numpy as np
import time

move = ["Idle", "wavehands", "jump", "frontback", "turnclap", "windowcleaning",
        "numbersix", "jumpleftright", "sidestep", "squatturnclap", "window360", "logout"]
clf = joblib.load('v_ultomato.pkl')

def predict_data(data_set):
    data_array = [row.split(',') for row in list(data_set)]
    data_array = [row[:-3] for row in data_array]
    data_array = [[float(number.replace('\x00','').replace('\n','')) for number in row] for row in data_array]
    # Features extraction
    numpy_data = np.array(data_array)
    mean = np.mean(numpy_data, axis=0)
    variance = np.var(numpy_data, axis=0)
    #median = np.median(numpy_data, axis=0)
    max_array = np.max(numpy_data, axis=0)
    min_array = np.min(numpy_data, axis=0)
    offset = max_array - min_array
    
    summ = np.array([])
    transposedcsv = np.transpose(numpy_data)

    for row in transposedcsv:
        n = len(row) # length of the signal
        Y = np.fft.fft(row) # fft computing and normalization
        Y = Y[range(int(n/2))]
        maxVal = np.amax(Y)

        indexOfMax = np.where(Y == maxVal)
        summ = np.append(summ, indexOfMax)
    
    x = np.append(mean[0:12], [variance[0:12], offset[0:12], summ[0:12]]).tolist()
    x = np.array(x).reshape(1,-1)
    prev_time = time.time()
    #print(move[int(clf.predict(x)[0])] + " in " + str(time.time() - prev_time) + "s")
    
    return move[int(clf.predict(x)[0])]

#predict_data(["1,2,3,4,5,3,1,2,3,4,5,3,10,11,13", "1,2,3,3,3,3,1,2,3,4,5,3,10,11,12"])
