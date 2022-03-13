import logging
logging.basicConfig(level=logging.DEBUG)

class PIDGains:
    def __init__(self):
        self._kp = 0.0
        self._ki = 0
        self._kd = 0
        
        self._kp_list=[]
        self._ki_list=[]
        self._kd_list=[]
    
    @property
    def kp(self):
        return self._kp

    @kp.setter
    def kp(self, val):
        if val >= 0:
            self._kp = val
        else:
            logging.warn("kp {} should be >=0".format(val))
            return

    @property
    def ki(self):
        return self._ki

    @ki.setter
    def ki(self, val):
        if val >= 0:
            self._ki = val
        else:
            logging.warn("ki {} should be >=0".format(val))
            return
    
    @property
    def kd(self):
        return self._kd

    @kd.setter
    def kd(self, val):
        if val >= 0:
            self._kd = val
        else:
            logging.warn("kd {} should be >=0".format(val))
            return

    def updateKpList(self):
        self._kp_list.append(self.kp)

    def updateKiList(self):
        self._ki_list.append(self.ki)

    def updateKdList(self):
        self._kd_list.append(self.kd)

    def updateLists(self):
        self._kp_list.append(self.kp)
        self._ki_list.append(self.ki)
        self._kd_list.append(self.kd)

    def resetLists(self):
        self._kp_list = []
        self._ki_list = []
        self._kd_list = []


def getPIDGainsFromCoeff(num=None, dt=0.001):
    """Computes Kp,Ki,Kd of a discrete PID controller from its transfer function's numerator coeff
    Numerator is of the form n0*Z^2+n1*Z+n2

    Parameters
    --
    @param num Numerator coeff =[n0,n1,n2]
    @param dt sampling time in seconds

    Returns
    --
    @return kp,ki,kd PID gains
    """
    kp=None
    ki=None
    kd=None
    if(num is None):
        logging.error("[getPIDGainsFromCoeff] numerator coeffs array is None")
        return kp, ki, kd
    if( dt<=0):
        logging.error("[getPIDGainsFromCoeff] sampling time should be >0")
        return kp, ki, kd

    n0=num[0]
    n1=num[1]
    n2=num[2]

    kd=dt*n2
    ki=(n0+n1+n2)/dt
    kp=(n0+n1-n2)/2.

    # Ref: last equation (13) in https://www.scilab.org/discrete-time-pid-controller-implementation
    # kp = (n0-n2)/2
    # ki = (n0+n1+n2)/(2.*dt)
    # kd = dt*(n0 - n1 + n2)/8.

    return kp,ki,kd

def getPIDCoeffFromGains(kp=None, ki=None, kd=None, dt=0.001):
    """Computes transfer function's numerator of a discrete PID from its gains.
    The denominator is constant Z^2+Z+0
    The numnerator is of the form n0*Z^2+n1*Z+n2

    Parameters
    --
    @param kp Proportional gain
    @param ki Integral gain
    @param kd Derivtive gain
    @param dt sampling time in seconds
    
    Returns
    --
    @return num=[n0,n1,n2] list of PID numerator coeffs
    """
    if kp is None or ki is None or kd is None:
        logging.error("[getPIDCoeffFromGains] One of the coeffs is None kp=%s, ki=%s, kd=%s", kp, ki, kd)
        return None

    if kp <0.0 or ki < 0.0 or kd<0.0:
        logging.error("[getPIDCoeffFromGains] One of the coeffs is negative, kp=%s, ki=%s, kd=%s", kp, ki, kd)
        return None
    if( dt<=0):
        logging.error("[getPIDCoeffFromGains] sampling time should be >0")
        return None

    # Numerator coeff
    n0=kp+ki*dt/2.+kd/dt
    n1=-kp+ki*dt/2.-2.*kd/dt
    n2=kd/dt

    # Ref: last equation (13) in https://www.scilab.org/discrete-time-pid-controller-implementation
    # n0 = (2*dt*kp + ki*dt**2 + 4*kd) /2./dt
    # n1 = (2*ki*dt**2 - 8*kd) / 2./dt
    # n2 = (ki*dt**2 - 2*dt*kp + 4*kd) /2./dt

    num=[n0,n1,n2]

    return num

def limitPIDGains(P=None, I=None, D=None, kp_min=0.0, kp_max=0.5, ki_min=0.0, ki_max=0.4, kd_min=0.0, kd_max=0.005):

    kP=None
    kI=None
    kD=None
    if P is None or I is None or D is None:
        logging.error("[limitPIDGains] One of the coeffs is None")
        return None
    kP=P
    kI=I
    kD=D
    
    if P < kp_min:
        kP=kp_min
    if P > kp_max:
        kP=kp_max

    if I<ki_min:
        kI=ki_min
    if I>ki_max:
        kI=ki_max

    if D<kd_min:
        kD=kd_min
    if D>kd_max:
        kD=kd_max

    return (kP, kI, kD)

def test():
    logging.info("\nThis is a test function.\n")