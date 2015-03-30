import wit

access_token = 'H35GRISJSBQOOOPT3WUNUJWEFELKNSF5'
wit.init()
response = wit.voice_query_auto(access_token)
print('Response: {}'.format(response))
wit.close()