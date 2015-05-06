import sys
sys.path.append("/home/esl2131/wit/pywit")
import wit
import json

#Returns command in a string, if no command found returns 'none'
def wit_start():
	access_token = 'UGWCDSIXPJLVZKUXFGWEE3NVCGLUW7EA'
	wit.init()
	response = wit.voice_query_auto(access_token)

	r = json.loads(response)
        txt = r['_text']
        if txt == 'null':
            return 'none',None
	important_results = r['outcomes']
	if important_results:
		confidence = important_results[0]['confidence']
		intent = important_results[0]['intent']
		recorded = important_results[0]['_text']
		
		if intent == 'stop':
			return 'stop', None

		if 'action_to_do' in important_results[0]['entities']:
			action = important_results[0]['entities']['action_to_do'][0]['value']
			if confidence > 0.5:
				#print 'recorded: ' + recorded 
				#print 'intent: ' + intent
				#print '    ' + action
				if intent == 'greeting' and action == 'wave':
					return 'wave', None
				elif intent == 'bump' and action == 'bump':
					return 'bump', None
				elif intent == 'follow' and action == 'follow':
					if 'color' in important_results[0]['entities']:
						color = important_results[0]['entities']['color'][0]['value']
						if color == 'red':
							c ='red'
						elif color == 'blue':
							c = 'blue'
						elif color == 'green':
							c = 'green'
						elif color == 'yellow':
							c = 'yellow'
						elif color == 'pink':
							c = 'pink'

						return 'follow', c
	return 'none', None
	wit.close()

if __name__ == '__main__':
    print wit_start()
