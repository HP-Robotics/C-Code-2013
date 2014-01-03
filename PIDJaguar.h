/*----------------------------------------------------------------------------
  **    The WPILib PIDController class is accustomed to be used for
  **    driving to a fixed position, not to a set rate.
  **    The PIDController.cpp Calculate() logic clearly intends
  **    for PIDWrite() to be called with a *delta* value; the goal
  **    is to eventually just call that with 0.  That makes sense
  **    when you're driving to a position.
  **    But the Jaguar class expects to be called with an absolute
  **    value.  So this code just wraps the Jaguar class and provides
  **    a PIDWrite() function that takes a delta.
*/
class PIDJaguar : public Jaguar
{
    private:
        float m_speed;
        float m_min_speed;
        float m_max_speed;
        Encoder *m_enc;

    public:
        PIDJaguar(UINT32 channel, float min_speed = -0.99, float max_speed = 0.0) : Jaguar(channel)
        {
            m_speed = 0;
            m_min_speed = min_speed;
            m_max_speed = max_speed;
        }


        void SetEncoder(Encoder *e)
        {
            m_enc = e;
        }
        void Set(float speed)
        {
            if (speed < m_min_speed)
                speed = m_min_speed;
            if (speed > m_max_speed)
                speed = m_max_speed;
            m_speed = speed;
            Jaguar::Set(speed);
        }

        void PIDWrite(float delta)
        {
            //printf("%g:  PIDWrite delta %f, to %f, rate %f\n", Timer::GetFPGATimestamp(), delta, m_speed + delta, m_enc->GetRate());
            Set(m_speed + delta);
        }

        void Feed(void)
        {
            Set(m_speed);
        }
};
