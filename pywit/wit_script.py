import sys
sys.path.append("/home/esl2131/wit/pywit")
import wit

access_token = 'UGWCDSIXPJLVZKUXFGWEE3NVCGLUW7EA'
wit.init()
response = wit.voice_query_auto(access_token)
print('Response: {}'.format(response))
wit.close()
