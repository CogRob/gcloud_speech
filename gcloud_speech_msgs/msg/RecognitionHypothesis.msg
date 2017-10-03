# Recognition hypothesis for a portion of speech. Equivalent to
# google.cloud.speech.v1.SpeechRecognitionAlternative, See also:
# https://cloud.google.com/speech/reference/rpc/google.cloud.speech.v1#speechrecognitionalternative

# Transcript text representing the words that the user spoke.
string transcript

# The confidence estimate between 0.0 and 1.0. A higher number indicates an
# estimated greater likelihood that the recognized words are correct. This field
# is typically provided only for the top hypothesis, and only for
# is_portion_final=true results. Clients should not rely on the confidence field
# as it is not guaranteed to be accurate or consistent. The default of 0.0 is a
# sentinel value indicating confidence was not set.
float64 confidence
