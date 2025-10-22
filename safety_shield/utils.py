import numpy

def list2str(lnList):
    length = len(lnList)
    ret = ""
    for i in range(length):
        ret += str(round(lnList[i]*100)/100)
        if i<length-1:
            ret += ", "

    return ret

def str2list(strList):
    out = []
    lst = strList.split(',')
    for s in lst:
        out.append(float(s.replace(' ', "")))
    
    return out


# python class to represnet quantizers
class Quantizer:
    def __init__(self, x_lb, x_eta, x_ub):
        self.x_lb = numpy.float32(x_lb)
        self.x_eta = numpy.float32(x_eta)
        self.x_ub = numpy.float32(x_ub)
        self.x_dim = len(x_eta)
        self.x_widths = [0]*self.x_dim
        self.num_symbols = 1

        for i in range(self.x_dim):
            tmp = numpy.floor((self.x_ub[i]-self.x_lb[i])/self.x_eta[i])+1
            self.x_widths[i]  = int(tmp)
            self.num_symbols = self.num_symbols * self.x_widths[i]

    def flat_to_conc(self, flat_val):
        fltInitial = flat_val

        ret = [0.0]*self.x_dim
        for i in range(self.x_dim):
            fltCurrent = fltInitial

            fltVolume = 1
            for k in range(i):
                fltTmp = self.x_widths[k]
                fltVolume = fltVolume*fltTmp

            fltCurrent = int(fltCurrent / fltVolume)
            fltTmp = self.x_widths[i]
            fltCurrent = int(fltCurrent%fltTmp)

            ret[i] = float(self.x_lb[i] + self.x_eta[i]*fltCurrent)

            fltCurrent = fltCurrent * fltVolume
            fltInitial = fltInitial - fltCurrent

        return ret

    def conc_to_flat(self, x_c):
        x_sym = [0.0]*self.x_dim
        x_conc = numpy.float32(x_c)

        for i in range(self.x_dim):
            x_sym[i] = int(numpy.floor((x_conc[i] - self.x_lb[i] + self.x_eta[i]/numpy.float32(2.0))/self.x_eta[i]))

        
        x_flat = 0
        for i in range(self.x_dim):
            fltTmpVolume = 1

            for j in range(i):
                fltTmpVolume *= self.x_widths[j]

            fltTmp = x_sym[i]
            fltTmp *= fltTmpVolume
            x_flat += fltTmp

        return x_flat

    def get_widths(self):
        return self.x_widths

    def get_num_symbols(self):
        return self.num_symbols